#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8

def qos_reliable(depth=10):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE
    )

class BumperSafety(Node):
    def __init__(self):
        super().__init__('bumper_safety')

        # Zeiten (x5) + Entsperrlogik
        self.declare_parameter('back_speed', -1.8)
        self.declare_parameter('back_time',  0.85)
        self.declare_parameter('turn_speed',  1.8)
        self.declare_parameter('turn_time',   0.9)
        self.declare_parameter('hold_time',   0.8)     # Sperrphase nach Ende
        self.declare_parameter('require_zero_to_resume', True)

        self.declare_parameter('raw_cmd_topic', '/cmd_vel_raw')
        self.declare_parameter('out_cmd_topic', '/cmd_vel')
        self.declare_parameter('bumper_topic', '/bumper/state')

        self.back_speed = float(self.get_parameter('back_speed').value)
        self.back_time  = float(self.get_parameter('back_time').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.turn_time  = float(self.get_parameter('turn_time').value)
        self.hold_time  = float(self.get_parameter('hold_time').value)
        self.require_zero_to_resume = bool(self.get_parameter('require_zero_to_resume').value)

        raw_cmd_topic = self.get_parameter('raw_cmd_topic').value
        out_cmd_topic = self.get_parameter('out_cmd_topic').value
        bumper_topic  = self.get_parameter('bumper_topic').value

        qos = qos_reliable()

        self.pub = self.create_publisher(Twist, out_cmd_topic, qos)
        self.sub_cmd = self.create_subscription(Twist, raw_cmd_topic, self.on_cmd, qos)
        self.sub_bmp = self.create_subscription(UInt8, bumper_topic, self.on_bumper, qos)

        self.timer = self.create_timer(0.01, self.on_timer)  # 100 Hz

        # State
        self.override_until = 0.0
        self.override_phase = None   # 'back' -> 'turn' -> None
        self.turn_dir = 0.0
        self.last_raw = Twist()

        self.block_raw_until = 0.0   # Sperrphase nach Ende
        self.wait_zero = False       # erst /cmd_vel_raw == 0/0 zulassen?
        self.zero_eps = 1e-3

        self.get_logger().info(
            f"BumperSafety ready. raw={raw_cmd_topic} out={out_cmd_topic} bumpers={bumper_topic} "
            f"[back={self.back_speed} m/s,{self.back_time}s | turn={self.turn_speed} rad/s,{self.turn_time}s | "
            f"hold={self.hold_time}s, require_zero={self.require_zero_to_resume}]"
        )

    def send_cmd(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

    def on_cmd(self, msg: Twist):
        self.last_raw = msg
        now = time.monotonic()

        # Während Override oder Sperrphase: nicht durchreichen
        if now < self.override_until or now < self.block_raw_until:
            return

        # Optional erst „Null“ verlangen, bevor wieder freigegeben wird
        if self.wait_zero:
            if abs(msg.linear.x) < self.zero_eps and abs(msg.angular.z) < self.zero_eps:
                self.wait_zero = False
                self.get_logger().info("Resume unlocked by ZERO command.")
            else:
                # Ignorieren bis 0/0 kommt
                return

        # Normal: direkt durchreichen
        self.pub.publish(msg)

    def on_bumper(self, msg: UInt8):
        m = msg.data
        if m == 0:
            return
        now = time.monotonic()
        if now < self.override_until:
            return

        # Richtung: Bit0=links -> rechts drehen (-), Bit1=rechts -> links drehen (+), beide -> rechts (-)
        if   (m & 0x01) and not (m & 0x02): self.turn_dir = -1.0
        elif (m & 0x02) and not (m & 0x01): self.turn_dir = +1.0
        else:                                self.turn_dir = -1.0

        # Override starten
        self.override_phase = 'back'
        self.override_until = now + self.back_time
        self.wait_zero = self.require_zero_to_resume  # nach Ende ggf. „Zero“ verlangen
        self.block_raw_until = 0.0                    # block wird am Ende gesetzt
        self.get_logger().warn(f"BUMPER 0x{m:02X} -> BACK {self.back_time:.2f}s, dann TURN {self.turn_time:.2f}s dir={self.turn_dir:+.0f}")

        # Sofort reagieren
        self.send_cmd(self.back_speed, 0.0)

    def on_timer(self):
        now = time.monotonic()

        # Override aktiv?
        if now < self.override_until:
            if self.override_phase == 'back':
                self.send_cmd(self.back_speed, 0.0)
            elif self.override_phase == 'turn':
                self.send_cmd(0.0, self.turn_dir * self.turn_speed)
            return

        # Phasenwechsel / Ende?
        if self.override_phase == 'back':
            self.override_phase = 'turn'
            self.override_until = now + self.turn_time
            self.send_cmd(0.0, self.turn_dir * self.turn_speed)
            return

        if self.override_phase == 'turn':
            # Manöver zu Ende -> explizit STOP senden
            self.send_cmd(0.0, 0.0)
            # Sperrphase aktivieren (ignoriert Rohkommandos für hold_time)
            self.block_raw_until = now + self.hold_time
            self.override_phase = None
            self.turn_dir = 0.0
            self.get_logger().info(f"Override done. Holding for {self.hold_time:.2f}s. "
                                   f"Require ZERO: {self.require_zero_to_resume}")
            return
        # Kein Override: nichts tun; on_cmd regelt Durchleitung

def main():
    rclpy.init()
    node = BumperSafety()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
