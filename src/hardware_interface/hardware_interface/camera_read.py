#-- The Script to start the camera and publish the stream --#

import serial
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class Camera_Node(Node):
    def __init__(self):
        super().__init__("camera_node")
        
        self.img_publisher=self.create_publisher(Image,"/camera/img",10)
        
        self.br=CvBridge()
        
        self.get_logger().info("Camera Activated")
        
        self.start_camera()
        
    # Function to read the camera stream and publish it to ros topic
    def start_camera(self):
        try:
            cap = cv2.VideoCapture(0)
            while True:
                ret, frame = cap.read()
                frame=cv2.resize(frame,(600,500),interpolation=cv2.INTER_LINEAR)
                frame=cv2.flip(frame,1)
                
                img_msg=self.br.cv2_to_imgmsg(frame)
                
                self.img_publisher.publish(img_msg)
                
                cv2.waitKey(1)
            
        except :
            cap.release()
            cv2.destroyAllWindows()
            self.get_logger().info("Problem with camera or no camera")



def main(args=None):
    rclpy.init(args=args)
    node=Camera_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__=="__main__":
    main()



