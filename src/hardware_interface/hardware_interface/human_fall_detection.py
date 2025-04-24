#-- The Script to handle and process camera data and get pose and related estimates --#

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import cv2
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_msgs.msg import PoseModelData


class PoseDetect(Node):
    def __init__(self):
        super().__init__("pose_detect_node")
        
        # Path to medianet pose detection model
        model_path = '/home/cerbrus/cerbrus_ws/src/hardware_interface/pose_landmarker_heavy.task'
        
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            output_segmentation_masks=True)
        self.detector = vision.PoseLandmarker.create_from_options(options)

        # Get the default frame width and height
        self.frame_width = 600
        self.frame_height = 500
        
        self.image_subscriber=self.create_subscription(Image,"/camera/img",self.get_pose,10)
        
        self.image_publisher=self.create_publisher(Image,"/camera/img_processed",10)
        
        self.pose_publisher=self.create_publisher(PoseModelData,"/camera/pose_msg",10)
        
        self.br=CvBridge()
        
        self.get_logger().info("Pose and Fall Detection Started")
        
    # Function to predict the pose
    def get_pose(self,msg):
        
        frame=self.br.imgmsg_to_cv2(msg)

        frame=cv2.flip(frame,1)
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
        detection_result = self.detector.detect(image)
        annotated_image = self.draw_points(frame, detection_result)
        
        annotated_img_msg=self.br.cv2_to_imgmsg(annotated_image)
                
        self.image_publisher.publish(annotated_img_msg)

            
    # Function to draw the points on the image
    def draw_points(self,img, detection_result):
        pose_landmarks_list = detection_result.pose_landmarks
        pose_msg=PoseModelData()
        pose_msg.dist=-1.0
        pose_msg.angle=-1.0
        pose_msg.head_pt=[-1,-1]
        pose_msg.mid_pt=[-1,-1]
        pose_msg.leg_pt=[-1,-1]

        # Averaging the upper points, middle points and lower points to a get 3 points.
        if len(pose_landmarks_list)!=0:
            upper_points=pose_landmarks_list[0][0:11]
            middle_points=pose_landmarks_list[0][11:15]+pose_landmarks_list[0][23:25]
            lower_points=pose_landmarks_list[0][27:]

            upper_x=[]
            upper_y=[]

            middle_x=[]
            middle_y=[]

            lower_x=[]
            lower_y=[]

            dist=0

            for i in upper_points:
                upper_x.append(i.x*self.frame_width)
                upper_y.append(i.y*self.frame_height)
                dist+=i.z
            dist=dist/len(upper_points)
            
            for i in middle_points:
               middle_x.append(i.x*self.frame_width)
               middle_y.append(i.y*self.frame_height)

            for i in lower_points:
               lower_x.append(i.x*self.frame_width)
               lower_y.append(i.y*self.frame_height)
       
            head_pt=(int(sum(upper_x)/len(upper_x)),int(sum(upper_y)/len(upper_y)))
            mid_pt=(int(sum(middle_x)/len(middle_x)),int(sum(middle_y)/len(middle_y)))
            leg_pt=(int(sum(lower_x)/len(lower_x)),int(sum(lower_y)/len(lower_y)))

            angle=self.get_angle(head_pt,leg_pt)
            
            
            
            pose_msg.dist=dist
            pose_msg.angle=angle
            pose_msg.head_pt=list(head_pt)
            pose_msg.mid_pt=list(mid_pt)
            pose_msg.leg_pt=list(leg_pt)
            
            self.pose_publisher.publish(pose_msg)

            cv2.circle(img,head_pt,20,(0,0,255),15)
            cv2.circle(img,mid_pt,20,(0,255,0),15)
            cv2.circle(img,leg_pt,20,(255,0,0),15)
            cv2.putText(img,f"Dist: {dist}",(30,30),cv2.FONT_HERSHEY_SIMPLEX,0.8,(255,255,0),2)
        
        self.pose_publisher.publish(pose_msg)
                

        return img
    
    # This function gets the orientation of human from the horizontal frame of camera(in degrees)
    def get_angle(self,pt_a1,pt_a2):

        try:
          m1=(pt_a1[1]-pt_a2[1])/(pt_a1[0]-pt_a2[0])
        except:
            return 90.0
        m2=0

        angle=math.degrees(math.atan2((m2-m1),(1+m1*m2)))


        return angle


def main(args=None):
    rclpy.init(args=args)
    node=PoseDetect()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()



if __name__=="__main__":
    main()
