#!/usr/bin/env python3
import math
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ShapeDriver(Node):
    def __init__(self, shape: str, side: float, v_lin: float, v_ang: float, topic: str):
        super().__init__('shape_cmdvel_timer')
        self.pub = self.create_publisher(Twist, topic, 10)

        # Parameter
        self.shape = shape.lower()
        self.n_sides = 4 if self.shape in ('square', 'viereck', 'rectangle') else 3
        self.side = float(side)
        self.v_lin = abs(float(v_lin))
        self.v_ang = abs(float(v_ang))

        # Zeit / Steuerung
        self.rate_hz = 20.0
        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self._on_timer)

        # Zustände
        self.edge_idx = 0
        self.phase = 'straight'  # 'straight' oder 'turn'
        self.phase_time = 0.0

        # Phasendauern
        # Zeit = Weg / Geschwindigkeit bzw. Winkel / Winkelgeschwindigkeit
        self.t_straight = self.side / max(self.v_lin, 1e-6)
        self.turn_angle = 2 * math.pi / self.n_sides
        self.t_turn = self.turn_angle / max(self.v_ang, 1e-6)

        self.get_logger().info(
            f"Fahre {self.n_sides}-Eck: side={self.side:.2f}, v_lin={self.v_lin:.2f}, "
            f"v_ang={self.v_ang:.2f}, topic={topic}"
        )

    def _publish(self, lin_x: float, ang_z: float):
        msg = Twist()
        msg.linear.x = lin_x
        msg.angular.z = ang_z
        self.pub.publish(msg)

    def _on_timer(self):
        # Phase abarbeiten
        if self.phase == 'straight':
            self._publish(self.v_lin, 0.0)
            self.phase_time += self.dt
            if self.phase_time >= self.t_straight:
                # Zur Drehphase wechseln
                self.phase = 'turn'
                self.phase_time = 0.0

        elif self.phase == 'turn':
            # Links herum drehen; für rechts herum: negatives v_ang
            self._publish(0.0, self.v_ang)
            self.phase_time += self.dt
            if self.phase_time >= self.t_turn:
                self.edge_idx += 1
                if self.edge_idx >= self.n_sides:
                    # Form fertig -> stoppen und Timer beenden
                    self._publish(0.0, 0.0)
                    self.get_logger().info("Form fertig. Stop.")
                    self.timer.cancel()
                    # Node später sauber beenden
                    rclpy.shutdown()
                    return
                # Nächste Kante
                self.phase = 'straight'
                self.phase_time = 0.0

def main():
    parser = argparse.ArgumentParser(description="Drive turtlesim in polygons using /cmd_vel.")
    parser.add_argument('--shape', default='square', help='square/viereck or triangle/dreieck')
    parser.add_argument('--side', type=float, default=2.0, help='side length (turtlesim units)')
    parser.add_argument('--v_lin', type=float, default=1.0, help='linear speed (m/s)')
    parser.add_argument('--v_ang', type=float, default=2.0, help='angular speed (rad/s)')
    parser.add_argument('--topic', default='/turtle1/cmd_vel', help='cmd_vel topic')
    args = parser.parse_args()

    rclpy.init()
    node = ShapeDriver(args.shape, args.side, args.v_lin, args.v_ang, args.topic)
    rclpy.spin(node)  # Timer steuert das Publizieren

if __name__ == '__main__':
    main()
