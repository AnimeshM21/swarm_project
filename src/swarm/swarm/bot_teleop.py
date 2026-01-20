#!/usr/bin/env python3

import rclpy
import sys, select, tty, termios, signal
from rclpy.node import Node
from std_msgs.msg import Float64


def clamp(v, lo, hi):
    return max(lo, min(v, hi))


def is_data():
    return select.select([sys.stdin], [], [], 0)[0]


class TeleopNode(Node):
    def __init__(self):
        super().__init__("bot_teleop")

        self.pubs = {
            'front_left_wheel': self.create_publisher(
                Float64, '/swarm/front_left_wheel/cmd_vel', 10),
            'front_right_wheel': self.create_publisher(
                Float64, '/swarm/front_right_wheel/cmd_vel', 10),
            'rear_left_wheel': self.create_publisher(
                Float64, '/swarm/rear_left_wheel/cmd_vel', 10),
            'rear_right_wheel': self.create_publisher(
                Float64, '/swarm/rear_right_wheel/cmd_vel', 10),
        }

        self.manual_offsets = {k: 0.0 for k in self.pubs}
        self.step = 1.0
        self.max_thrust = 50.0

        self.create_timer(0.1, self.publish_all)

    def publish_all(self):
        for name, pub in self.pubs.items():
            val = clamp(self.manual_offsets[name],
                        -self.max_thrust,
                        self.max_thrust)
            msg = Float64()
            msg.data = float(val)
            pub.publish(msg)

    def stop_all(self):
        for k in self.manual_offsets:
            self.manual_offsets[k] = 0.0
        self.publish_all()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    print("""
-------- Ground Bot Teleop --------
Arrow Keys : Move
W/S        : Fast Forward / Backward
A/D        : Sharp Turn
SPACE      : Stop
X          : Exit
""")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    def exit_clean(*_):
        node.stop_all()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, exit_clean)

    try:
        while rclpy.ok():
            if is_data():
                ch = sys.stdin.read(1)

                if ch == '\x1b':  # Arrow keys
                    seq = sys.stdin.read(2)

                    if seq == '[A':  # forward
                        for k in node.manual_offsets:
                            node.manual_offsets[k] += node.step

                    elif seq == '[B':  # backward
                        for k in node.manual_offsets:
                            node.manual_offsets[k] -= node.step

                    elif seq == '[C':  # right turn
                        node.manual_offsets['front_left_wheel'] += node.step
                        node.manual_offsets['rear_left_wheel'] += node.step
                        node.manual_offsets['front_right_wheel'] -= node.step
                        node.manual_offsets['rear_right_wheel'] -= node.step

                    elif seq == '[D':  # left turn
                        node.manual_offsets['front_left_wheel'] -= node.step
                        node.manual_offsets['rear_left_wheel'] -= node.step
                        node.manual_offsets['front_right_wheel'] += node.step
                        node.manual_offsets['rear_right_wheel'] += node.step

                else:
                    ch = ch.lower()

                    if ch == 'w':
                        for k in node.manual_offsets:
                            node.manual_offsets[k] += 2 * node.step

                    elif ch == 's':
                        for k in node.manual_offsets:
                            node.manual_offsets[k] -= 2 * node.step

                    elif ch == 'a':
                        node.manual_offsets['front_left_wheel'] -= 2 * node.step
                        node.manual_offsets['rear_left_wheel'] -= 2 * node.step
                        node.manual_offsets['front_right_wheel'] += 2 * node.step
                        node.manual_offsets['rear_right_wheel'] += 2 * node.step

                    elif ch == 'd':
                        node.manual_offsets['front_left_wheel'] += 2 * node.step
                        node.manual_offsets['rear_left_wheel'] += 2 * node.step
                        node.manual_offsets['front_right_wheel'] -= 2 * node.step
                        node.manual_offsets['rear_right_wheel'] -= 2 * node.step

                    elif ch == ' ':
                        node.stop_all()

                    elif ch == 'x':
                        exit_clean()

            rclpy.spin_once(node, timeout_sec=0.02)

    finally:
        exit_clean()


if __name__ == "__main__":
    main()
