#!/usr/bin/env python3

import rclpy
import sys, select, tty, termios, signal
from rclpy.node import Node
from geometry_msgs.msg import Twist

def is_data():
    return select.select([sys.stdin], [], [], 0)[0]

class SwarmTeleopNode(Node):
    def __init__(self):
        super().__init__("swarm_teleop")

        # Configuration
        self.num_bots = 3  # Change this if you spawn more bots
        self.active_bot = 1 # Start controlling bot_1
        
        self.publishers_dict = {}
        self.bot_states = {}

        # Initialize publishers and states for each bot
        for i in range(1, self.num_bots + 1):
            topic_name = f'/bot_{i}/cmd_vel'
            self.publishers_dict[i] = self.create_publisher(Twist, topic_name, 10)
            
            # State: [Linear Velocity, Angular Velocity]
            self.bot_states[i] = {'lin': 0.0, 'ang': 0.0}
            print(f"Initialized control for Bot {i} on topic: {topic_name}")

        # Step sizes
        self.linear_step = 0.5
        self.angular_step = 0.5
        self.max_linear = 5.0
        self.max_angular = 5.0

        # Timer to keep publishing commands for ALL bots (persistence)
        self.create_timer(0.1, self.publish_all_cmds)

    def publish_all_cmds(self):
        """Continuously publish the last known state for EVERY bot."""
        for i in range(1, self.num_bots + 1):
            msg = Twist()
            msg.linear.x = self.bot_states[i]['lin']
            msg.angular.z = self.bot_states[i]['ang']
            self.publishers_dict[i].publish(msg)

    def stop_active_bot(self):
        self.bot_states[self.active_bot]['lin'] = 0.0
        self.bot_states[self.active_bot]['ang'] = 0.0

    def stop_all_bots(self):
        for i in range(1, self.num_bots + 1):
            self.bot_states[i]['lin'] = 0.0
            self.bot_states[i]['ang'] = 0.0
        print("\r\n*** STOPPED ALL BOTS ***")

def main(args=None):
    rclpy.init(args=args)
    node = SwarmTeleopNode()

    print(f"""
-------- SWARM TELEOP (DiffDrive) --------
Current Active Bot: {node.active_bot}

CONTROLS:
1 - {node.num_bots}     : Switch Active Bot
W / Up      : Increase Speed (Active Bot)
S / Down    : Decrease Speed (Active Bot)
A / Left    : Turn Left (Active Bot)
D / Right   : Turn Right (Active Bot)
SPACE       : Stop Active Bot
0           : EMERGENCY STOP ALL
X           : Exit
------------------------------------------
""")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    def exit_clean(*_):
        node.stop_all_bots()
        # Publish one last time to ensure stop commands go through
        node.publish_all_cmds()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, exit_clean)

    try:
        while rclpy.ok():
            if is_data():
                ch = sys.stdin.read(1)
                
                # Check for Number Keys (Bot Selection)
                if ch.isdigit() and ch != '0':
                    bot_idx = int(ch)
                    if 1 <= bot_idx <= node.num_bots:
                        node.active_bot = bot_idx
                        # Print generic status to avoid clutter
                        print(f"\rSwitched to Bot {node.active_bot} | ", end="")
                
                # Emergency Stop All
                elif ch == '0':
                    node.stop_all_bots()

                # Arrow keys
                elif ch == '\x1b':
                    seq = sys.stdin.read(2)
                    if seq == '[A':   # Up
                        node.bot_states[node.active_bot]['lin'] += node.linear_step
                    elif seq == '[B': # Down
                        node.bot_states[node.active_bot]['lin'] -= node.linear_step
                    elif seq == '[C': # Right
                        node.bot_states[node.active_bot]['ang'] -= node.angular_step
                    elif seq == '[D': # Left
                        node.bot_states[node.active_bot]['ang'] += node.angular_step

                else:
                    ch = ch.lower()
                    if ch == 'w':
                        node.bot_states[node.active_bot]['lin'] += node.linear_step
                    elif ch == 's':
                        node.bot_states[node.active_bot]['lin'] -= node.linear_step
                    elif ch == 'a':
                        node.bot_states[node.active_bot]['ang'] += node.angular_step
                    elif ch == 'd':
                        node.bot_states[node.active_bot]['ang'] -= node.angular_step
                    elif ch == ' ':
                        node.stop_active_bot()
                    elif ch == 'x':
                        exit_clean()

                # Clamp values for the ACTIVE bot
                current_lin = node.bot_states[node.active_bot]['lin']
                current_ang = node.bot_states[node.active_bot]['ang']
                
                current_lin = max(min(current_lin, node.max_linear), -node.max_linear)
                current_ang = max(min(current_ang, node.max_angular), -node.max_angular)
                
                node.bot_states[node.active_bot]['lin'] = current_lin
                node.bot_states[node.active_bot]['ang'] = current_ang

                # Print status
                print(f"\r[Bot {node.active_bot}] Lin: {current_lin:.2f} | Ang: {current_ang:.2f}   ", end="")

            rclpy.spin_once(node, timeout_sec=0.02)

    finally:
        exit_clean()

if __name__ == "__main__":
    main()