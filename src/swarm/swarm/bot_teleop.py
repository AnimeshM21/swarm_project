#!/usr/bin/env python3

import rclpy
import sys, select, tty, termios, signal
from rclpy.node import Node
from geometry_msgs.msg import Twist  # We now use Twist for DiffDrive

def is_data():
    return select.select([sys.stdin], [], [], 0)[0]

class TeleopNode(Node):
    def __init__(self):
        super().__init__("bot_teleop")

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.target_linear = 0.0
        self.target_angular = 0.0
        
        # Step sizes
        self.linear_step = 1.0
        self.angular_step = 1.0
        self.max_linear = 20.0
        self.max_angular = 20.0

        # Timer to keep publishing the command
        self.create_timer(0.1, self.publish_cmd)

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.target_linear
        msg.angular.z = self.target_angular
        self.pub.publish(msg)

    def stop_all(self):
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.publish_cmd()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    print("""
-------- Ground Bot Teleop (DiffDrive) --------
W / Arrow Up    : Increase Speed
S / Arrow Down  : Decrease Speed
A / Arrow Left  : Turn Left
D / Arrow Right : Turn Right
SPACE           : Stop
X               : Exit
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

                # Arrow keys
                if ch == '\x1b':
                    seq = sys.stdin.read(2)
                    if seq == '[A':   # Up
                        node.target_linear += node.linear_step
                    elif seq == '[B': # Down
                        node.target_linear -= node.linear_step
                    elif seq == '[C': # Right
                        node.target_angular -= node.angular_step
                    elif seq == '[D': # Left
                        node.target_angular += node.linear_step

                else:
                    ch = ch.lower()
                    if ch == 'w':
                        node.target_linear += node.linear_step
                    elif ch == 's':
                        node.target_linear -= node.linear_step
                    elif ch == 'a':
                        node.target_angular += node.angular_step
                    elif ch == 'd':
                        node.target_angular -= node.angular_step
                    elif ch == ' ':
                        node.target_linear = 0.0
                        node.target_angular = 0.0
                    elif ch == 'x':
                        exit_clean()

                # Clamp values
                node.target_linear = max(min(node.target_linear, node.max_linear), -node.max_linear)
                node.target_angular = max(min(node.target_angular, node.max_angular), -node.max_angular)

                # Print status
                print(f"\rLin: {node.target_linear:.2f} | Ang: {node.target_angular:.2f}", end="")

            rclpy.spin_once(node, timeout_sec=0.02)

    finally:
        exit_clean()

if __name__ == "__main__":
    main()