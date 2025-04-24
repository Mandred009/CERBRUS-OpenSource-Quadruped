#-- The Script to handle the fall detection that is follow the person if he has fallen--#

import numpy as np
import cv2
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_msgs.msg import PoseModelData, JoyStick


class AutoFallCmd(Node):
    def __init__(self):
        super().__init__("fall_commander_node")
        

        # Get the default frame width and height
        self.frame_width = 600
        self.frame_height = 500
        
        self.center_of_frame=(self.frame_width//2,self.frame_height//2)
        
        
        self.pose_subscriber=self.create_subscription(PoseModelData,"/camera/pose_msg",self.pose_callback,10)
        
        self.auto_joy_publisher=self.create_publisher(JoyStick,"/joystick/auto",10)
        
        self.get_logger().info("Fall Commander Started")
        
    
    def pose_callback(self,msg):
        
        auto_j=JoyStick()
        auto_j.jx=500
        auto_j.jy=500
        auto_j.button=True
        
        
        dist=msg.dist
        angle=msg.angle
        head_pt=msg.head_pt
        mid_pt=msg.mid_pt
        leg_pt=msg.leg_pt

        if head_pt[0]!=-1 and dist>-0.9 and angle<=20: # follow person if he is a bit far from bot and if his angle is less than 20 that is he is most likely fallen down.
            auto_j.jy=0
        
        self.auto_joy_publisher.publish(auto_j)
        
        
        


def main(args=None):
    rclpy.init(args=args)
    node=AutoFallCmd()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()



if __name__=="__main__":
    main()

