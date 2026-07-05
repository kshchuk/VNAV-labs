#!/usr/bin/env python3
"""Bridge Gazebo Classic quadrotor physics to Lab 3 controller topics."""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Wrench
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from rclpy.node import Node

# Match controller_node.cpp physical constants.
CF = 1e-3
CD = 1e-5
D = 0.3
A = CF * D / math.sqrt(2.0)

F2W = np.array([
    [CF, CF, CF, CF],
    [A, A, -A, -A],
    [-A, A, A, -A],
    [CD, -CD, CD, -CD],
])


class QuadrotorBridge(Node):
    def __init__(self):
        super().__init__('quadrotor_bridge')
        self.declare_parameter('odom_topic', '/quadrotor/ground_truth/odometry')
        self.declare_parameter('wrench_topic', '/quadrotor/wrench')

        odom_topic = self.get_parameter('odom_topic').value
        wrench_topic = self.get_parameter('wrench_topic').value

        self._wrench_pub = self.create_publisher(Wrench, wrench_topic, 10)
        self._state_pub = self.create_publisher(Odometry, '/current_state', 10)

        self.create_subscription(Actuators, 'rotor_speed_cmds', self._on_cmd, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        self.get_logger().info(
            f'Bridge active: {odom_topic} -> /current_state, '
            f'rotor_speed_cmds -> {wrench_topic}'
        )

    def _on_odom(self, msg: Odometry):
        self._state_pub.publish(msg)

    def _on_cmd(self, msg: Actuators):
        if len(msg.angular_velocities) < 4:
            return

        speeds = np.array(msg.angular_velocities[:4], dtype=float)
        # Thrust/torque map uses signed squared rotor speeds (controller eq. 1).
        signed_square = speeds * np.abs(speeds)
        wrench = F2W @ signed_square

        out = Wrench()
        out.force.x = 0.0
        out.force.y = 0.0
        out.force.z = float(wrench[0])
        out.torque.x = float(wrench[1])
        out.torque.y = float(wrench[2])
        out.torque.z = float(wrench[3])
        self._wrench_pub.publish(out)


def main():
    rclpy.init()
    node = QuadrotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
