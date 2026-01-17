#!/usr/bin/env python3

import rclpy 
import sys,select, tty, termios,signal
from rclpy.node import Node
from std_msgs.msg import Float64

def clamp(v,lo,hi):
    return max(lo,min(v,hi))

    def is_data():
        return select.select([sys.stdin],[],[],0) == ([sys.stdin],[],[])

class TeleopNode(Node):
    def __init(self):
        super().__init__("bot_teleop")

        self.pubs = {
            'front_left_wheel':self.create_publisher(
                Float64,'/swarm/front_left_wheel/cmd_vel',10),
            'front_right_wheel':self.create_publisher(
                Float64,'/swarm/front_right_wheel/cmd_vel',10),
            'rear_left_wheel':self.create_publisher(
                Float64,'/swarm/rear_left_wheel/cmd_vel',10),
            'rear_right_wheel':self.create_publisher(
                Float64,'/swarm/rear_right_wheel/cmd_vel',10),                
        }

        self.manual_offsets = {k:0.0 for k in self.pubs.keys()}

        self.step = 1.0
        self.max_thrust = 50.0

        self.timer = self.create_timer(0.1, self.publish_all)


    def publish_all(self):
        values = self.manual_offsets[name]
        values= clamp(values, -self.max_thrust, self.max_thrust)

        msg = Float64()
        msg.data = float(values)
        pub.publish(msg)

    def stop_all(self):
        for k in self.manual_offsets:
            self.manual_offsets[k] = 0.0
        self.publish_all()

    def main(args=None):
        rclpy.init(args=args)
        node = TeleopNode()

        print("""
        --------Ground Bot Teleop--------
        
        Keyboard Input Needed

        Controls:
        Arrow Up / Arrow Down     : Forward / Backward
        Arrow Left / Arrow Right  : Mild Left Turn / Mild Right Turn

        W / S                     : Fast Forward / Fast Backward
        A / D                     : Steep Left Turn / Steep Right Turn

        SPACE                     : Set all speed to 0
        X                         : Exit
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
                    ch = sys.stdin.read(2)

                if ch == '\x1b':
                    seq = sys.stdin.read(2)

                    if seq == '[A':  # forward
                        node.manual_offsets['left_front_wheel'] += node.step
                        node.manual_offsets['left_rear_wheel'] += node.step
                        node.manual_offsets['right_front_wheel'] += node.step
                        node.manual_offsets['right_rear_wheel'] += node.step

                    elif seq == '[B':  # backward
                        node.manual_offsets['left_front_wheel'] -= node.step
                        node.manual_offsets['left_rear_wheel'] -= node.step
                        node.manual_offsets['right_front_wheel'] -= node.step
                        node.manual_offsets['right_rear_wheel'] -= node.step

                    elif seq == '[C':  #mild right turn
                        node.manual_offsets['left_front_wheel'] += node.step
                        node.manual_offsets['left_rear_wheel'] += node.step
                        node.manual_offsets['right_front_wheel'] -= node.step
                        node.manual_offsets['right_rear_wheel'] -= node.step

                    elif seq == '[D': #mild left turn
                        node.manual_offsets['left_front_wheel'] -= node.step
                        node.manual_offsets['left_rear_wheel'] -= node.step
                        node.manual_offsets['right_front_wheel'] += node.step
                        node.manual_offsets['right_rear_wheel'] += node.step  

                else:
                    ch = ch.lower()

                    if ch == 'w':  # fast forward
                        node.manual_offsets['left_front_wheel'] += 2*node.step
                        node.manual_offsets['left_rear_wheel'] += 2*node.step
                        node.manual_offsets['right_front_wheel'] += 2*node.step
                        node.manual_offsets['right_rear_wheel'] += 2*node.step  

                    elif ch == 'a':  # fast backward
                        node.manual_offsets['left_front_wheel'] -= 2*node.step
                        node.manual_offsets['left_rear_wheel'] -= 2*node.step
                        node.manual_offsets['right_front_wheel'] -= 2*node.step
                        node.manual_offsets['right_rear_wheel'] -= 2*node.step  

                    elif ch == 's':  # sharp right turn 
                        node.manual_offsets['left_front_wheel'] += 2*node.step
                        node.manual_offsets['left_rear_wheel'] += 2*node.step
                        node.manual_offsets['right_front_wheel'] -= 2*node.step
                        node.manual_offsets['right_rear_wheel'] -= 2*node.step  

                    elif ch == 'd':  # fast left turn 
                        node.manual_offsets['left_front_wheel'] -= 2*node.step
                        node.manual_offsets['left_rear_wheel'] -= 2*node.step
                        node.manual_offsets['right_front_wheel'] += 2*node.step
                        node.manual_offsets['right_rear_wheel'] += 2*node.step   

                    elif ch == ' ':
                        node.stop_all()

                    elif ch == 'x':
                        exit_clean()

                for k in node.manual_offsets:
                    node.manual_offsets[k] = clamp(
                        node.manual_offsets[k],
                        -node.max_thrust,
                        node.max_thrust
                    )

            rclpy.spin_once(node, timeout_sec=0.02)

    finally:
        exit_clean()


if __name__ == "__main__":
    main()