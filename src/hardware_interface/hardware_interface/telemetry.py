#-- The Script to handle the telemetry data --#

import rclpy
from rclpy.node import Node
from custom_msgs.msg import IMU
from std_msgs.msg import String, Int16, Float32
import serial
import time
import ast
import subprocess
import socket

class TelemetryNode(Node): # Node for telemetry
    def __init__(self):
        super().__init__("telemetry_node")
        
        self.sensor_comm_ard_port="/dev/ttyUSB1" # Connects to telemetry arduino nano
        
        imu_topic="/imu_data"
        cmd_topic="/commander_data"
        volt_topic="/voltage/battery"
        mode_topic="/mode"
        
        self.imu_subscriber=self.create_subscription(IMU,imu_topic,self.imu_callback,10)
        self.cmd_subscriber=self.create_subscription(String,cmd_topic,self.cmd_callback,10)
        self.volt_subscriber=self.create_subscription(Float32,volt_topic,self.volt_callback,10)
        self.mode_subscriber=self.create_subscription(Int16,mode_topic,self.mode_callback,10)
        
        self.ser=serial.Serial(self.sensor_comm_ard_port, 9600)
        time.sleep(2)
        
        # Start command signals the nano to start the telemetry receiving
        for i in range(0,10):
            self.ser.write(f"start\n".encode())
            time.sleep(0.2)
        
        self.get_logger().info("TELEMETRY READY")
        
        
        self.create_timer(0.6,self.send_ard_data)
        
        self.imu_data=""
        self.cmd_data=""
        self.volt_data=""
        self.mode_data=""
        
        self.wifi_name=self.get_wifi_name().strip()
        self.ip_address=self.get_ip().strip()
    
    # Telemetry data format is values separated by ',' and enclosed in '()'.
    def send_ard_data(self):
        self.wifi_name=self.get_wifi_name().strip()
        self.ip_address=self.get_ip().strip()
        self.ser.write(f"({self.wifi_name},{self.ip_address},{self.imu_data},{self.cmd_data},{self.volt_data},{self.mode_data})\n".encode())
        
    def imu_callback(self,msg):
        self.imu_data=f"{msg.pitch},{msg.roll},{msg.yaw}".strip()
        
    def cmd_callback(self,msg):
        self.cmd_data=f"{msg.data}".strip()
    
    def volt_callback(self,msg):
        self.volt_data=f"{round(msg.data,2)}".strip()
        
    def mode_callback(self,msg):
        self.mode_data=f"{msg.data}".strip()
        
    def get_wifi_name(self): # Get the name of currently connected Wi-Fi
        try:
            output=subprocess.check_output(['iwgetid','-r']).decode('utf-8').strip()
            return output
        except:
            return "ERR"
    def get_ip(self): # Get the ip of currently connected Wi-Fi
        try:
            s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            s.connect(('10.255.255.255',1))
            ip=s.getsockname()[0]
            s.close()
            return ip
        except:
            return "ERR"

def main(args=None):
    rclpy.init(args=args)
    node=TelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__=="__main__":
    main()

