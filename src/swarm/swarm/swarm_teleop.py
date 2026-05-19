#!/usr/bin/env python3
"""
Unified swarm teleop — controls any combination of ground bots and drones.

Reads num_bots / num_drones from ROS 2 parameters (set by swarm.launch.py).
Agents are numbered sequentially:
    1 … num_bots           → ground bots  (publish Twist to /bot_N/cmd_vel)
    num_bots+1 … total     → drones       (publish Twist to /drone[_N]/cmd_vel
                                            + Bool to /drone[_N]/enable)

Controls:
    1–9        Switch active agent
    W/S        Forward / Backward   (ground bot: linear.x,  drone: linear.x)
    A/D        Turn Left / Right    (ground bot: angular.z,  drone: angular.z yaw)
    Arrow ↑/↓  ground bot: same as W/S; drone: Ascend / Descend (linear.z)
    Arrow ←/→  ground bot: same as A/D; drone: Strafe left / right (linear.y)
    SPACE      Stop active agent
    0          Emergency stop ALL
    X          Quit
"""

import rclpy
import sys
import select
import tty
import termios
import signal
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


def _is_data():
    return select.select([sys.stdin], [], [], 0)[0]


class SwarmTeleopNode(Node):
    def __init__(self):
        super().__init__('swarm_teleop')

        # ── Parameters ───────────────────────────────────────
        self.declare_parameter('num_bots', 3)
        self.declare_parameter('num_drones', 0)
        # Always wall-clock: teleop timers must fire even when sim is paused.
        self.declare_parameter('use_sim_time', False)
        self.num_bots   = self.get_parameter('num_bots').value
        self.num_drones = self.get_parameter('num_drones').value
        self.total      = self.num_bots + self.num_drones

        if self.total == 0:
            self.get_logger().error('num_bots + num_drones = 0, nothing to control.')
            return

        # ── Per-agent state ──────────────────────────────────
        # agent_id 1..total; first num_bots are ground bots, rest are drones
        self.active = 1

        # Ground bot state: {agent_id: {'lin': float, 'ang': float}}
        # Drone state:      {agent_id: {'x': float, 'y': float, 'z': float, 'yaw': float}}
        self.bot_pubs   = {}
        self.bot_states = {}
        self.drone_pubs    = {}   # Twist publisher
        self.drone_en_pubs = {}   # Bool publisher
        self.drone_states  = {}

        for i in range(1, self.num_bots + 1):
            topic = f'/bot_{i}/cmd_vel'
            self.bot_pubs[i] = self.create_publisher(Twist, topic, 10)
            self.bot_states[i] = {'lin': 0.0, 'ang': 0.0}
            self.get_logger().info(f'Agent {i} → ground bot on {topic}')

        for j in range(1, self.num_drones + 1):
            agent_id = self.num_bots + j
            if self.num_drones == 1:
                ns = 'drone'
            else:
                ns = f'drone_{j}'
            self.drone_pubs[agent_id]    = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
            self.drone_en_pubs[agent_id] = self.create_publisher(Bool, f'/{ns}/enable', 10)
            self.drone_states[agent_id]  = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
            self.get_logger().info(f'Agent {agent_id} → drone on /{ns}/cmd_vel')

        # Speed tuning
        self.bot_linear_step  = 0.5
        self.bot_angular_step = 0.5
        self.bot_max_linear   = 5.0
        self.bot_max_angular  = 5.0

        self.drone_step      = 1.0
        self.drone_max_speed = 10.0

        # Persistent publish timer
        self.create_timer(0.1, self._publish_all)

        # Arm all drones after a short delay so subscribers are ready
        if self.num_drones > 0:
            self._arm_timer = self.create_timer(1.5, self._arm_all_drones)

    # ── Publishing ───────────────────────────────────────────

    def _publish_all(self):
        for i, state in self.bot_states.items():
            msg = Twist()
            msg.linear.x  = state['lin']
            msg.angular.z = state['ang']
            self.bot_pubs[i].publish(msg)

        for i, state in self.drone_states.items():
            msg = Twist()
            msg.linear.x  = state['x']
            msg.linear.y  = state['y']
            msg.linear.z  = state['z']
            msg.angular.z = state['yaw']
            self.drone_pubs[i].publish(msg)

    def _arm_all_drones(self):
        msg = Bool()
        msg.data = True
        for pub in self.drone_en_pubs.values():
            pub.publish(msg)
        self.get_logger().info(f'Armed {self.num_drones} drone(s)')
        self._arm_timer.cancel()  # fire once only

    # ── Agent queries ────────────────────────────────────────

    def _is_drone(self, agent_id):
        return agent_id > self.num_bots

    def _agent_label(self, agent_id):
        if self._is_drone(agent_id):
            j = agent_id - self.num_bots
            return f'Drone {j}' if self.num_drones > 1 else 'Drone'
        return f'Bot {agent_id}'

    # ── Input handling ───────────────────────────────────────

    def handle_key(self, ch):
        a = self.active

        if self._is_drone(a):
            s = self.drone_states[a]
            # Arrow keys for drone: ↑↓ = altitude, ←→ = strafe
            if ch == 'UP':
                s['z'] += self.drone_step
            elif ch == 'DOWN':
                s['z'] -= self.drone_step
            elif ch == 'LEFT':
                s['y'] += self.drone_step
            elif ch == 'RIGHT':
                s['y'] -= self.drone_step
            elif ch == 'w':
                s['x'] += self.drone_step
            elif ch == 's':
                s['x'] -= self.drone_step
            elif ch == 'a':
                s['yaw'] += self.drone_step
            elif ch == 'd':
                s['yaw'] -= self.drone_step
            # clamp
            for k in s:
                s[k] = max(min(s[k], self.drone_max_speed), -self.drone_max_speed)
        else:
            s = self.bot_states[a]
            if ch in ('UP', 'w'):
                s['lin'] += self.bot_linear_step
            elif ch in ('DOWN', 's'):
                s['lin'] -= self.bot_linear_step
            elif ch in ('LEFT', 'a'):
                s['ang'] += self.bot_angular_step
            elif ch in ('RIGHT', 'd'):
                s['ang'] -= self.bot_angular_step
            s['lin'] = max(min(s['lin'], self.bot_max_linear), -self.bot_max_linear)
            s['ang'] = max(min(s['ang'], self.bot_max_angular), -self.bot_max_angular)

    def stop_active(self):
        a = self.active
        if self._is_drone(a):
            self.drone_states[a] = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        else:
            self.bot_states[a] = {'lin': 0.0, 'ang': 0.0}

    def stop_all(self):
        for i in self.bot_states:
            self.bot_states[i] = {'lin': 0.0, 'ang': 0.0}
        for i in self.drone_states:
            self.drone_states[i] = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        self._publish_all()

    def _status_line(self):
        a = self.active
        label = self._agent_label(a)
        if self._is_drone(a):
            s = self.drone_states[a]
            return (f'[{label}] X:{s["x"]:+.1f} Y:{s["y"]:+.1f} '
                    f'Z:{s["z"]:+.1f} Yaw:{s["yaw"]:+.1f}')
        else:
            s = self.bot_states[a]
            return f'[{label}] Lin:{s["lin"]:+.2f} Ang:{s["ang"]:+.2f}'


def main(args=None):
    rclpy.init(args=args)
    node = SwarmTeleopNode()

    if node.total == 0:
        rclpy.shutdown()
        return

    banner = f"""
══════════════════════════════════════════════
  SWARM TELEOP  │  {node.num_bots} bot(s)  +  {node.num_drones} drone(s)
══════════════════════════════════════════════
  1–{node.total}          Select agent
  W/S           Forward / Backward
  A/D           Rotate left / right
  ↑/↓           Bot: fwd/back  │ Drone: up/down
  ←/→           Bot: turn      │ Drone: strafe
  SPACE         Stop active agent
  0             EMERGENCY STOP ALL
  X             Quit
──────────────────────────────────────────────
"""
    print(banner)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    def exit_clean(*_):
        node.stop_all()
        # Disarm drones
        if node.num_drones > 0:
            msg = Bool()
            msg.data = False
            for pub in node.drone_en_pubs.values():
                pub.publish(msg)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.get_logger().info('Shutting down swarm teleop.')
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, exit_clean)

    try:
        while rclpy.ok():
            if _is_data():
                ch = sys.stdin.read(1)

                # Arrow keys
                if ch == '\x1b':
                    seq = sys.stdin.read(2)
                    arrow_map = {'[A': 'UP', '[B': 'DOWN', '[C': 'RIGHT', '[D': 'LEFT'}
                    ch = arrow_map.get(seq)
                    if ch is None:
                        continue
                elif ch.isdigit():
                    idx = int(ch)
                    if idx == 0:
                        node.stop_all()
                        print('\r\n*** EMERGENCY STOP ALL ***')
                    elif 1 <= idx <= node.total:
                        node.active = idx
                        print(f'\r  → Switched to {node._agent_label(idx):<20}', end='')
                    continue
                else:
                    ch = ch.lower()
                    if ch == ' ':
                        node.stop_active()
                    elif ch == 'x':
                        exit_clean()
                    elif ch not in ('w', 'a', 's', 'd'):
                        continue

                if ch:
                    node.handle_key(ch)

                print(f'\r  {node._status_line():<60}', end='')

            rclpy.spin_once(node, timeout_sec=0.02)

    finally:
        exit_clean()


if __name__ == '__main__':
    main()
