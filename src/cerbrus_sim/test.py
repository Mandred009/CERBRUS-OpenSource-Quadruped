#!/usr/bin/env python3
"""
Save as: test_mujoco_ros.py
Run with: python3 test_mujoco_ros.py
"""

import mujoco
import mujoco.viewer
import numpy as np
from pathlib import Path
import threading
import time

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class JointCommandReceiver(Node):
    def __init__(self):
        super().__init__('joint_cmd_receiver')
        
        self.cmd = np.array([0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6])
        self.lock = threading.Lock()
        
        self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.callback,
            10
        )
        self.get_logger().info('Subscriber created on /joint_commands')
    
    def callback(self, msg):
        print(f'\n*** CALLBACK RECEIVED: {msg.data} ***\n')
        if len(msg.data) == 12:
            with self.lock:
                self.cmd = np.array(msg.data)
    
    def get_cmd(self):
        with self.lock:
            return self.cmd.copy()


def main():
    # Initialize ROS
    rclpy.init()
    node = JointCommandReceiver()
    
    # Spin ROS in background thread
    def spin():
        print('ROS spin thread started')
        rclpy.spin(node)
        print('ROS spin thread ended')
    
    ros_thread = threading.Thread(target=spin, daemon=True)
    ros_thread.start()
    
    # Give ROS time to initialize
    time.sleep(1.0)
    
    # Check if subscriber is active
    print(f'Node name: {node.get_name()}')
    print(f'Subscriptions: {node.subscriptions}')
    
    # Load MuJoCo
    model_path = Path(__file__).parent / 'urdf' / 'mjmodel.xml'
    print(f'Loading: {model_path}')
    
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    
    # Set initial ctrl
    data.ctrl[:] = node.get_cmd()
    
    print('\n' + '='*50)
    print('SIMULATION RUNNING')
    print('='*50)
    print('\nIn another terminal, run:')
    print('ros2 topic pub --once /joint_commands std_msgs/Float64MultiArray "data: [0.0, 1.2, -2.2, 0.0, 1.2, -2.2, 0.0, 1.2, -2.2, 0.0, 1.2, -2.2]"')
    print('\n')
    
    # Run simulation
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # Apply command
            data.ctrl[:] = node.get_cmd()
            
            # Step
            mujoco.mj_step(model, data)
            
            # Sync
            viewer.sync()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()