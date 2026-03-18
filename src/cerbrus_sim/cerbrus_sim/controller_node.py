#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose

import numpy as np


class CerbrusControllerNode(Node):
    def __init__(self):
        super().__init__('cerbrus_controller')
        
        # Parameters
        self.declare_parameter('control_rate', 100.0)
        control_rate = self.get_parameter('control_rate').value
        
        # Standing pose target
        self.standing_pose = np.array([
            0.0, 0.8, -1.6,  # Front Left
            0.0, 0.8, -1.6,  # Front Right
            0.0, 0.8, -1.6,  # Back Left
            0.0, 0.8, -1.6,  # Back Right
        ])
        
        # Current state
        self.current_joint_pos = np.zeros(12)
        self.current_body_pose = None
        
        # Publisher
        self.cmd_pub = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        self.body_pose_sub = self.create_subscription(
            Pose,
            'body_pose',
            self.body_pose_callback,
            10
        )
        
        # Control timer
        self.create_timer(1.0 / control_rate, self.control_loop)
        
        self.get_logger().info('Cerbrus controller node started')
    
    def joint_state_callback(self, msg: JointState):
        """Update current joint positions."""
        self.current_joint_pos = np.array(msg.position)
    
    def body_pose_callback(self, msg: Pose):
        """Update current body pose."""
        self.current_body_pose = msg
    
    def control_loop(self):
        """Main control loop - send standing commands."""
        cmd = Float64MultiArray()
        cmd.data = self.standing_pose.tolist()
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CerbrusControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()