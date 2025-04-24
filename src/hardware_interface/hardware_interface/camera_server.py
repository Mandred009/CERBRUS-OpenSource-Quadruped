#-- The Script to start a simple camera server. Optional script --#

import serial
import time
import rclpy
from rclpy.node import Node
import cv2
import socket
import pickle
import struct

class Camera_Node(Node):
    def __init__(self):
        super().__init__("camera_node_server")
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('0.0.0.0', 8888))
        self.server_socket.listen(5)
        self.get_logger().info("Server is listening...")
        self.client_socket, self.client_address = self.server_socket.accept()
        self.get_logger().info(f"Connection from {self.client_address} accepted")
        
        self.send_stream()
        
    def send_stream(self):
        try:
            cap = cv2.VideoCapture(0)
            while True:
                ret, frame = cap.read()
                frame=cv2.resize(frame,(300,200),interpolation=cv2.INTER_LINEAR)
                frame_data = pickle.dumps(frame)
                self.client_socket.sendall(struct.pack("Q", len(frame_data)))
                self.client_socket.sendall(frame_data)
                
                cv2.waitKey(1)
            
        except :
            cap.release()
            cv2.destroyAllWindows()
            self.get_logger().info("Problem with stream or client disconnected")



def main(args=None):
    rclpy.init(args=args)
    node=Camera_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__=="__main__":
    main()


