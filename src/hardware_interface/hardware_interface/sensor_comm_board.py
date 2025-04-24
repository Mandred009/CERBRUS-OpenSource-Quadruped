#-- The Script that takes the sensor values and interfaces the sensor suite arduino --#

import rclpy
from rclpy.node import Node
from custom_msgs.msg import IMU, JoyStick, ButtonLeft, ButtonRight, Potentiometer
from std_msgs.msg import Float32,Int16
import serial
import time
import ast
import subprocess

class CommunicationSensorNode(Node): # Node to interface sensor suite Arduino
    def __init__(self):
        super().__init__("communication_sensor_node")
        
        self.sensor_comm_ard_port="/dev/ttyUSB0"
        
        # Publisher and subscriber topics
        imu_topic="/imu_data"
        cmd_topic="/commander_data"
        joystick_topic_r="/joystick/right"
        joystick_topic_l="/joystick/left"
        voltage_topic="/voltage/battery"
        mode_topic="/mode"
        auto_joy_topic="/joystick/auto"
        button_left_topic="/buttons/left"
        button_right_topic="/buttons/right"
        potentiometer_topic="/potentiometer"
        
        self.imu_publisher=self.create_publisher(IMU,imu_topic,10)
        self.joystick_publisher_r=self.create_publisher(JoyStick,joystick_topic_r,10)
        self.joystick_publisher_l=self.create_publisher(JoyStick,joystick_topic_l,10)
        self.volt_publisher=self.create_publisher(Float32,voltage_topic,10)
        self.mode_publisher=self.create_publisher(Int16,mode_topic,10)
        self.button_left_publisher=self.create_publisher(ButtonLeft,button_left_topic,10)
        self.button_right_publisher=self.create_publisher(ButtonRight,button_right_topic,10)
        self.potentiometer_publisher=self.create_publisher(Potentiometer,potentiometer_topic,10)
        
        self.auto_joy_subscriber=self.create_subscription(JoyStick,auto_joy_topic,self.auto_joy_callback,10)
        
        # Start the serial interface and wait for 2 seconds
        self.ser=serial.Serial(self.sensor_comm_ard_port, 9600)
        time.sleep(2)

        self.get_logger().info("SENSOR SUITE PUBLISHERS READY")
        
        # Checks if the main nodes are ready that is switch is turned on the controller
        self.nodes_start=0
        
        # 0 for manual joystick 1 for autonomous cmd
        self.mode=0 
        
        # Auto over-ride for manual joysticks
        self.auto_joy=JoyStick()
        self.auto_joy.jx=500
        self.auto_joy.jy=500
        self.auto_joy.button=True
        
        
        self.create_timer(0.0001,self.get_ard_data)
        
    
    def auto_joy_callback(self,msg): # Callback to get the auto_joystick values
        self.auto_joy=msg
    
    
    # Func to get the data from sensors on arduino
    def get_ard_data(self):
        while True:
            if self.ser.in_waiting>0:
                try:
                    data=self.ser.readline().decode()
                except:
                    continue 
                self.process_data(data)
                break
    
    # Function to process the in coming arduino data
    def process_data(self,data):

        try:
            points=data.split(",")
            imu_msg=IMU()
            joy_msg_r=JoyStick()
            joy_msg_l=JoyStick()
            volt_msg=Float32()
            mode_msg=Int16()
            button_left_msg=ButtonLeft()
            button_right_msg=ButtonRight()
            poten_msg=Potentiometer()

            
            imu_msg.yaw=float(points[1])
            imu_msg.pitch=float(points[2])
            imu_msg.roll=float(points[3])
            
            volt_msg.data=round(float(points[4]),2)
            
            mode_msg.data=self.mode           
            
            joy_vals=points[0][1:-1].split("|")
            
            if joy_vals[-1]=="on": # if the state is on then nodes_start is 1
                self.nodes_start=1
            
            if joy_vals[-1]=="off" and self.nodes_start==1: # if state is off then raspberry is shut-down
                self.nodes_start=0
                subprocess.call("./shut_cerb.sh")
            
            if self.mode==0:
                joy_msg_r.jx=int(joy_vals[0])
                joy_msg_r.jy=int(joy_vals[1])
                joy_msg_r.button=bool(int(joy_vals[2]))
                
            elif self.mode==1:
                joy_msg_r.jx=int(self.auto_joy.jx)
                joy_msg_r.jy=int(self.auto_joy.jy)
                joy_msg_r.button=bool(int(joy_vals[2]))
            
            joy_msg_l.jx=int(joy_vals[3])
            joy_msg_l.jy=int(joy_vals[4])
            joy_msg_l.button=bool(int(joy_vals[5]))
            
            button_left_msg.a=bool(int(joy_vals[6]))
            button_left_msg.b=bool(int(joy_vals[7]))
            button_left_msg.c=bool(int(joy_vals[8]))
            button_left_msg.d=bool(int(joy_vals[9]))
            
            if button_left_msg.d==False:
                self.mode=1-self.mode
            
            button_right_msg.one=bool(int(joy_vals[10]))
            button_right_msg.two=bool(int(joy_vals[11]))
            button_right_msg.three=bool(int(joy_vals[12]))
            button_right_msg.four=bool(int(joy_vals[13]))
            
            poten_msg.pot_r=int(joy_vals[14])
            poten_msg.pot_l=int(joy_vals[15])
            
            self.imu_publisher.publish(imu_msg)
            self.volt_publisher.publish(volt_msg)
            self.joystick_publisher_r.publish(joy_msg_r)
            self.joystick_publisher_l.publish(joy_msg_l)
            self.button_left_publisher.publish(button_left_msg)
            self.button_right_publisher.publish(button_right_msg)
            self.mode_publisher.publish(mode_msg)
            self.potentiometer_publisher.publish(poten_msg)
            
        except:
            return
        
        

def main(args=None):
    rclpy.init(args=args)
    node=CommunicationSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__=="__main__":
    main()
