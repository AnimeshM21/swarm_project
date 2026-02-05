#!/usr/bin/env python3
import rclpy
import sys, select, tty, termios, signal
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from std_msgs.msg import Bool

def is_data():
    return select.select([sys.stdin], [], [], 0)[0]

class TeleopNode(Node):
    def __init__(self):
        super().__init__("drone_teleop")
        
        self.pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        
        self.arm_pub = self.create_publisher(Bool, '/drone/enable', 10)

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0

        self.step = 1.0 
        self.max_speed = 30.0

        self.create_timer(0.1, self.publish_cmd)

        self.arm_drone(True)

    def arm_drone(self, state):
        msg = Bool()
        msg.data = state
        self.arm_pub.publish(msg)
        self.get_logger().info(f"Sending Arming Signal: {state}")

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.target_x
        msg.linear.y = self.target_y
        msg.linear.z = self.target_z
        msg.angular.z = self.target_yaw
        self.pub.publish(msg)

    def stop_all(self):
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0
        self.publish_cmd()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    print("""
-------- Drone Flight Control --------
    [W] / [S]        : Forward / Backward (X)
    [Arrow LEFT/RGHT]: Strafe Left / Right (Y)
    [Arrow UP/DOWN]  : Ascend / Descend (Z)
    [A] / [D]        : Rotate Left / Right (Yaw)
    
    SPACE            : HOVER (Sets all velocities to 0)
    X                : Quit & Stop
--------------------------------------
""")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    def exit_clean(*_):
        node.stop_all()
        node.arm_drone(False)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.get_logger().info("Shutting down teleop...")
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, exit_clean)

    try:
        while rclpy.ok():
            if is_data():
                ch = sys.stdin.read(1)

                if ch == '\x1b': 
                    seq = sys.stdin.read(2)
                    if seq == '[A':   node.target_z += node.step # Up Arrow
                    elif seq == '[B': node.target_z -= node.step # Down Arrow
                    elif seq == '[C': node.target_y -= node.step # Right Arrow
                    elif seq == '[D': node.target_y += node.step # Left Arrow
                else:
                    ch = ch.lower()
                    if ch == 'w':     node.target_x += node.step
                    elif ch == 's':   node.target_x -= node.step
                    elif ch == 'a':   node.target_yaw += node.step
                    elif ch == 'd':   node.target_yaw -= node.step
                    elif ch == ' ':   node.stop_all()
                    elif ch == 'x':   exit_clean()

                for attr in ['target_x', 'target_y', 'target_z', 'target_yaw']:
                    val = getattr(node, attr)
                    setattr(node, attr, max(min(val, node.max_speed), -node.max_speed))

                print(f"\rCMD -> X: {node.target_x:.1f} | Y: {node.target_y:.1f} | Z: {node.target_z:.1f} | Yaw: {node.target_yaw:.1f}   ", end="")

            rclpy.spin_once(node, timeout_sec=0.01)

    finally:
        exit_clean()

if __name__ == "__main__":
    main()