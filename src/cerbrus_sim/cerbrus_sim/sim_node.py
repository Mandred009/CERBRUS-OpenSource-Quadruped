#!/usr/bin/env python3

import mujoco
import mujoco.viewer
import numpy as np
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


class CerbrusSimNode(Node):
    def __init__(self):
        super().__init__('cerbrus_sim')
        
        pkg_path = Path(get_package_share_directory('cerbrus_sim'))
        model_path = pkg_path / 'urdf' / 'mjmodel.xml'
        
        self.get_logger().info(f'Loading: {model_path}')
        
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "standing")
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        
        self.cmd = np.array([0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6])
        self.lock = threading.Lock()
        
        self.joint_names = [
            'front_left_b_joint', 'front_left_ul_joint', 'front_left_ll_joint',
            'front_right_b_joint', 'front_right_ul_joint', 'front_right_ll_joint',
            'back_left_b_joint', 'back_left_ul_joint', 'back_left_ll_joint',
            'back_right_b_joint', 'back_right_ul_joint', 'back_right_ll_joint',
        ]
        
        self.create_subscription(Float64MultiArray, 'joint_commands', self.cmd_callback, 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.pose_pub = self.create_publisher(Pose, 'body_pose', 10)
        
        self.get_logger().info('Ready!')
    
    def cmd_callback(self, msg):
        self.get_logger().info(f'Received: {[round(x,2) for x in msg.data]}')
        if len(msg.data) == 12:
            with self.lock:
                self.cmd = np.array(msg.data)
    
    def get_cmd(self):
        with self.lock:
            return self.cmd.copy()
    
    def publish_state(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = self.data.qpos[7:].tolist()
        js.velocity = self.data.qvel[6:].tolist()
        self.joint_pub.publish(js)
        
        pose = Pose()
        pose.position.x = self.data.qpos[0]
        pose.position.y = self.data.qpos[1]
        pose.position.z = self.data.qpos[2]
        pose.orientation.w = self.data.qpos[3]
        pose.orientation.x = self.data.qpos[4]
        pose.orientation.y = self.data.qpos[5]
        pose.orientation.z = self.data.qpos[6]
        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = CerbrusSimNode()
    
    # ROS spin in background thread
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()
    
    time.sleep(0.5)
    
    try:
        step_count = 0
        with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
            while viewer.is_running():
                node.data.ctrl[:] = node.get_cmd()
                mujoco.mj_step(node.model, node.data)
                
                step_count += 1
                if step_count >= 10:
                    node.publish_state()
                    step_count = 0
                
                viewer.sync()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()