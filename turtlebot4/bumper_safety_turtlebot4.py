#!/usr/bin/env python3
"""
BumperSafety-Node für TurtleBot 4 (Create 3 Base).

Funktion:
- Abonniert  /cmd_vel_raw         (Twist)                -> Teleop-/Planer-Befehle
- Abonniert  /hazard_detection    (HazardDetectionVector)-> Bumper-Events vom TurtleBot4
- Publiziert /cmd_vel             (Twist)                -> an die Base

Logik:
- NORMAL: /cmd_vel_raw wird direkt auf /cmd_vel weitergeleitet.
- Bei Bumper:
    1) Rückwärts (BACK)
    2) Wegdrehen (TURN)
    3) Explizites STOP und kurze Sperrphase (HOLD)
    4) Zurück zu NORMAL, optional erst nach 0/0-Kommando.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection


# ---------- QoS-Helfer ----------

def qos_reliable(depth: int = 10) -> QoSProfile:
    """RELIABLE QoS, z.B. für /cmd_vel."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def qos_best_effort(depth: int = 10) -> QoSProfile:
    """BEST_EFFORT QoS, z.B. für Sensor-Topics wie /hazard_detection."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


# ---------- Zustände der kleinen Zustandsmaschine ----------

MODE_NORMAL = "NORMAL"
MODE_BACK   = "BACK"
MODE_TURN   = "TURN"
MODE_HOLD   = "HOLD"


class BumperSafety(Node):
    def __init__(self):
        super().__init__('bumper_safety')

        # ---------- Parameter ----------
        # Sanftere Default-Werte
        self.declare_parameter('back_speed', -0.18)   # m/s rückwärts
        self.declare_parameter('back_time',  1.2)     # s rückwärts
        self.declare_parameter('turn_speed',  0.9)    # rad/s drehen
        self.declare_parameter('turn_time',   1.4)    # s drehen
        self.declare_parameter('hold_time',   0.5)    # s Sperrphase danach
        self.declare_parameter('require_zero_to_resume', True)

        self.declare_parameter('raw_cmd_topic', '/cmd_vel_raw')
        self.declare_parameter('out_cmd_topic', '/cmd_vel')
        self.declare_parameter('hazard_topic', '/hazard_detection')
        self.declare_parameter('control_rate_hz', 50.0)  # 50 Hz reichen in der Praxis völlig

        p = self.get_parameter
        self.back_speed = float(p('back_speed').value)
        self.back_time  = float(p('back_time').value)
        self.turn_speed = float(p('turn_speed').value)
        self.turn_time  = float(p('turn_time').value)
        self.hold_time  = float(p('hold_time').value)
        self.require_zero_to_resume = bool(p('require_zero_to_resume').value)

        raw_cmd_topic = p('raw_cmd_topic').value
        out_cmd_topic = p('out_cmd_topic').value
        hazard_topic  = p('hazard_topic').value
        rate_ctrl     = max(float(p('control_rate_hz').value), 5.0)

        # ---------- QoS ----------
        qos_cmd = qos_reliable()
        qos_haz = qos_best_effort()

        # ---------- Publisher ----------
        self.pub = self.create_publisher(Twist, out_cmd_topic, qos_cmd)

        # ---------- Subscriber ----------
        # Teleop/Planner-Befehle
        self.sub_cmd = self.create_subscription(
            Twist, raw_cmd_topic, self.on_cmd_raw, qos_cmd
        )

        # Hazard-Events (Bumper etc.) vom TurtleBot4
        self.sub_haz = self.create_subscription(
            HazardDetectionVector, hazard_topic, self.on_hazard, qos_haz
        )

        # ---------- Timer (Control-Loop) ----------
        self.timer = self.create_timer(1.0 / rate_ctrl, self.on_timer)

        # ---------- interner Zustand ----------
        self.mode       = MODE_NORMAL
        self.until_time = 0.0          # Zeitmarke für Ende der aktuellen Phase
        self.turn_dir   = 0.0          # +1 = links, -1 = rechts
        self.last_raw   = Twist()      # letztes Teleop-Kommando
        self.wait_zero  = False        # ob 0/0-Kommando für Freigabe nötig ist
        self.zero_eps   = 1e-3         # Toleranz für "praktisch 0"

        # Debug-Rate für Logs (damit nicht jede Timer-Iteration loggt)
        self._dbg_tick = 0
        self._dbg_skip = int(rate_ctrl / 5.0)  # ~5 Debug-Meldungen pro Sekunde in Manöver-Phasen

        self.get_logger().info(
            f"BumperSafety aktiv.\n"
            f"Topics: RAW={raw_cmd_topic}, OUT={out_cmd_topic}, HAZ={hazard_topic}\n"
            f"Manöver: back={self.back_speed} m/s, {self.back_time} s | "
            f"turn={self.turn_speed} rad/s, {self.turn_time} s | "
            f"hold={self.hold_time} s | require_zero={self.require_zero_to_resume} | "
            f"ctrl_rate={rate_ctrl} Hz"
        )

    # ---------- Helper ----------

    def _now(self) -> float:
        return time.monotonic()

    def send_cmd(self, v: float, w: float) -> None:
        """Twist publizieren (linear.x = v, angular.z = w)."""
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

    # ---------- Callback: /cmd_vel_raw ----------

    def on_cmd_raw(self, msg: Twist) -> None:
        """Teleop-/Planner-Kommandos. In NORMAL durchreichen, sonst blocken."""
        self.last_raw = msg

        # Nur im Normalbetrieb werden Kommandos weitergegeben
        if self.mode != MODE_NORMAL:
            return

        # Optional: erst 0/0-Befehl verlangen (z.B. nach einem Bumper-Manöver)
        if self.wait_zero:
            if abs(msg.linear.x) < self.zero_eps and abs(msg.angular.z) < self.zero_eps:
                self.wait_zero = False
                self.get_logger().info("Teleop wieder freigegeben (0/0 erhalten).")
            else:
                # Noch nicht 0/0 -> nichts durchlassen
                return

        # Debug: gelegentlich loggen, was durchgeht
        # (nicht jede Nachricht, sonst wird die Konsole irre voll)
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.get_logger().debug(
                f"Pass-Through: v={msg.linear.x:.2f} m/s, w={msg.angular.z:.2f} rad/s"
            )

        self.pub.publish(msg)

    # ---------- Callback: /hazard_detection ----------

    def on_hazard(self, msg: HazardDetectionVector) -> None:
        """Hazard-Events auswerten, insbesondere Bumper."""
        detections = msg.detections
        if not detections:
            return

        # Nur reagieren, wenn wir im Normalbetrieb sind
        if self.mode != MODE_NORMAL:
            return

        # Bumper-Events herausfiltern
        bumps = [d for d in detections if d.type == HazardDetection.BUMP]
        if not bumps:
            return

        frames = [b.header.frame_id for b in bumps]

        # sehr simple Heuristik:
        left_hit   = any('left'  in f for f in frames)
        right_hit  = any('right' in f for f in frames)
        front_hit  = any('front' in f for f in frames)

        # Drehrichtung wählen:
        # - links getroffen -> nach rechts drehen (-1)
        # - rechts oder front -> nach links drehen (+1)
        if left_hit and not right_hit:
            self.turn_dir = -1.0
        elif right_hit or front_hit:
            self.turn_dir = +1.0
        else:
            self.turn_dir = -1.0  # fallback

        # Rückwärtsphase starten
        now = self._now()
        self.mode       = MODE_BACK
        self.until_time = now + self.back_time
        self.wait_zero  = self.require_zero_to_resume

        self.get_logger().warn(
            f"BUMPER erkannt: frames={frames} -> BACK {self.back_time:.2f}s, "
            f"dann TURN {self.turn_time:.2f}s (dir={self.turn_dir:+.0f})"
        )

        # Sofort reagieren
        self.send_cmd(self.back_speed, 0.0)

    # ---------- Timer: führt das Manöver aus ----------

    def on_timer(self) -> None:
        """Regelmäßiger Steuerloop (z.B. 50 Hz) für das Bumper-Manöver."""
        now = self._now()
        self._dbg_tick += 1

        # Je nach Modus unterschiedlich handeln
        if self.mode == MODE_BACK:
            if now < self.until_time:
                self.send_cmd(self.back_speed, 0.0)
                if self._dbg_tick % self._dbg_skip == 0:
                    self.get_logger().debug("Phase BACK: rückwärts.")
            else:
                # Übergang zu TURN
                self.mode       = MODE_TURN
                self.until_time = now + self.turn_time
                self.send_cmd(0.0, self.turn_dir * self.turn_speed)
                self.get_logger().info("Wechsel zu TURN-Phase.")

        elif self.mode == MODE_TURN:
            if now < self.until_time:
                self.send_cmd(0.0, self.turn_dir * self.turn_speed)
                if self._dbg_tick % self._dbg_skip == 0:
                    self.get_logger().debug("Phase TURN: drehen.")
            else:
                # STOP und in HOLD gehen
                self.send_cmd(0.0, 0.0)
                self.mode       = MODE_HOLD
                self.until_time = now + self.hold_time
                self.get_logger().info(
                    f"Manöver fertig. HOLD für {self.hold_time:.2f}s, "
                    f"require_zero={self.require_zero_to_resume}"
                )

        elif self.mode == MODE_HOLD:
            # während HOLD werden Teleop-Kommandos unterdrückt
            if now >= self.until_time:
                # zurück in NORMAL
                self.mode = MODE_NORMAL
                self.get_logger().info("Zurück in NORMAL-Modus (Teleop frei, ggf. mit 0/0-Sperre).")

        elif self.mode == MODE_NORMAL:
            # Hier macht der Timer nichts; Teleop-Pfade (on_cmd_raw) regeln alles.
            pass


def main() -> None:
    rclpy.init()
    node = BumperSafety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown (Ctrl-C).")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

