#-- The Main Central Control Script --#
#-1. z axis means in the direction along the plane cutting the face in half vertically
#-2. y axis means in the direction along the plane perpendicular to z axis
#-3. x or height axis means in the direction along the plane cutting the face in half horizontally

import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_msgs.msg import IMU,JoyStick, Potentiometer
import ast
import struct
from utilities.gait_traj import trot_swing
from utilities.pose import pose_change
from utilities.pid_pose_correction import PID
import math

class Commander_Node(Node): # Commander node for controlling driver
    def __init__(self):
        super().__init__("commander_node")
        
        self.joy_x_center=[400,700]
        self.joy_y_center=[400,700]
        
        
        self.bot_width=300.0
        self.bot_length=500.0
        
        self.max_roll=20 # in degrees
        self.max_pitch=20 # in degrees
        
        # Publisher and subscriber topics
        topic="/commander_data"
        imu_topic="/imu_data"
        joystick_topic_r="/joystick/right"
        joystick_topic_l="/joystick/left"
        poten_topic="/potentiometer"
        
        self.ard_port = '/dev/ttyACM0'
        
        self.cmd_publisher=self.create_publisher(String,topic,10)
        
        self.joystick_subscriber_r=self.create_subscription(JoyStick,joystick_topic_r,self.joystick_callback_r,10)
        self.joystick_subscriber_l=self.create_subscription(JoyStick,joystick_topic_l,self.joystick_callback_l,10)
        self.imu_subscriber=self.create_subscription(IMU,imu_topic,self.imu_callback,10)
        self.poten_subscriber=self.create_subscription(Potentiometer,poten_topic,self.poten_callback,10)
        
        # Opening the driver serial port and waiting a second for proper opening
        self.ser=serial.Serial(self.ard_port, 1000000)
        time.sleep(1)
        
        # Initializing the base and effector servos of the mouth
        self.base_poten=0.0
        self.effector_poten=0.0
        
        # 0 is stable 1 is trot forward 2 trot back
        self.state=0 
        
        # Initial height and leg pose arrays. Change these to effect the initial stand attitude
        self.init_ht=[-320,-320.0,-320.0,-320.0]
        self.init_z=[0.0,0.0,0.0,0.0]
        self.init_y=[0.0,0.0,0.0,0.0]
        self.init_vel=[2000.0,2000.0,2000.0,2000.0]
        
        self.leg_moving=[0,0,0,0]
        
        # Arrays to keep track of current pose
        self.current_ht=self.init_ht
        self.current_z=self.init_z
        self.current_y=self.init_y
        self.current_vel=self.init_vel
        
        # The height while hovering
        self.hover_ht=self.init_ht
        
        # Change these parameters to change the gait length and height
        self.step_length_z=40.0 # forward
        self.step_length_y=20.0 # sideways
        self.step_height=30.0
        
        # variables to store joystick mapping
        self.z_map=0
        self.y_map=0
        
        # Arrays that contain the final pose
        self.final_ht=[x + self.step_height for x in self.init_ht]
        self.final_z=[x + self.step_length_z for x in self.init_z]
        self.final_y=[x + self.step_length_y for x in self.init_y]
        self.final_vel=[3500.0,3500.0,3500.0,3500.0]

        
        self.yaw=0.0
        self.pitch=0.0
        self.roll=0.0
        
        # The ideal pitch and roll you want the bot to maintain while standing
        self.idle_pitch=0.0
        self.idle_roll=0.0
        
        # P,D,I values for pitch controller
        kp_p=3.0
        kd_p=6.25
        ki_p=0.0095
        
        # P,D,I values for roll controller
        kp_r=1.5
        kd_r=5.2
        ki_r=0.01
        
        # Creating a PID controller object
        self.pid=PID(self.idle_pitch,self.idle_roll,kp_p,kd_p,ki_p,kp_r,kd_r,ki_r,self.bot_width,self.bot_length)
        
        # Timing the main loop and timer
        self.time_step=0
        self.time_increment=(2*math.pi)/650 # 0.00966 sec
        
        self.phase_cnt=0
        self.motion_flag=0
        
        self.trot_flag=1
        self.hover_flag=1
        
        self.hover=False
        self.start_balance=True
        
        self.idle()
        
        self.state=0
        
        # Multiplier to specify the impact of PID controller while walking or moving
        self.multiplier=0.35
        
 
        self.get_logger().info("Central Nervous System Ready")
        
        self.create_timer(self.time_increment,self.publish_cmd)
        
        
    def poten_callback(self,msg): # Callback to process potentiometer values from controller
        self.base_poten=float(msg.pot_r)
        self.effector_poten=float(msg.pot_l)
    
    
    def joystick_callback_r(self,msg): # Callback to process right joystick values
        if msg.button==False:
            self.start_balance=bool(1-self.start_balance)
            time.sleep(1.2)

        # Logic to map the joystick to motion values
        if msg.jx>self.joy_x_center[1]:
            self.y_map=self.step_length_y
        elif msg.jx<self.joy_x_center[0]:
            self.y_map=self.step_length_y*-1
        else:
            self.y_map=0
            
        if msg.jy>self.joy_y_center[1]:
            self.z_map=self.step_length_z*-1
        elif msg.jy<self.joy_y_center[0]:
            self.z_map=self.step_length_z
        else:
            self.z_map=0
            
            
        if self.z_map==0 and self.y_map==0:
            self.state=0
        else:
            self.state=-1
    
    def joystick_callback_l(self,msg): # Callback to process left joystick values
        x_val=msg.jx
        y_val=msg.jy
        
        # Logic for hovering the bot
        if msg.button==False:
            self.hover=True
            if self.hover_ht[0]<=-360:
                self.hover_flag=-1
            elif self.hover_ht[0]>=-270:
                self.hover_flag=1
            
            self.hover_ht=[x-self.hover_flag for x in self.hover_ht]
        else:
            self.hover=False
            self.hover_ht=self.init_ht
        
        
        self.current_ht=pose_change(self.init_ht,x_val,y_val,self.bot_width,self.bot_length,self.max_roll,self.max_pitch)
        
    def imu_callback(self,msg): # Callback to process the IMU values
        self.yaw=msg.yaw
        self.pitch=msg.pitch
        self.roll=msg.roll
    
    
    def publish_cmd(self): # Main func called by the main timer
        msg=String()
        msg.data=f"{self.start_balance}"
        self.cmd_publisher.publish(msg) # Publish the balancer state
        
        if self.state==0:     
            self.idle(0)
            
        else: #[2,3] leg then [1,4]
            #[h,z,y,v]

            # Get the current point from the trajectory generator
            t_swing=trot_swing(self.time_step,self.z_map,self.y_map,self.step_height,self.final_vel[0],self.init_vel[0])
            t_stance=[0,0,0,0]

            if self.motion_flag==0: # Motion flag checks for the first step
                self.current_ht=self.pid.get_hts(self.roll,self.pitch,self.init_ht,[self.multiplier,0,0,self.multiplier])
                h=[self.current_ht[0],self.current_ht[1]+t_swing[0],self.current_ht[2]+t_swing[0],self.current_ht[3]]
                z=[self.current_z[0],t_swing[1],t_swing[1],self.current_z[3]]
                y=[self.current_y[0],t_swing[2],t_swing[2],self.current_y[3]]
                v=[t_swing[3]]*4
                
            else:
            
                if self.trot_flag==0: #[1,4] 1st
                    self.current_ht=self.pid.get_hts(self.roll,self.pitch,self.init_ht,[0,self.multiplier,self.multiplier,0])
                    h=[self.current_ht[0]+t_swing[0],self.current_ht[1]-t_stance[0],self.current_ht[2]-t_stance[0],self.current_ht[3]+t_swing[0]]
                    z=[t_swing[1],t_stance[1],t_stance[1],t_swing[1]]
                    y=[-t_swing[2],t_stance[2],t_stance[2],-t_swing[2]]
                    v=[t_swing[3],t_stance[3],t_stance[3],t_swing[3]]
                
                else: #[2,3] next
                    self.current_ht=self.pid.get_hts(self.roll,self.pitch,self.init_ht,[self.multiplier,0,0,self.multiplier])
                    h=[self.current_ht[0]-t_stance[0],self.current_ht[1]+t_swing[0],self.current_ht[2]+t_swing[0],self.current_ht[3]-t_stance[0]]
                    z=[t_stance[1],t_swing[1],t_swing[1],t_stance[1]]
                    y=[t_stance[2],t_swing[2],t_swing[2],t_stance[2]]
                    v=[t_stance[3],t_swing[3],t_swing[3],t_stance[3]]

            # Creating a command list to send to controller
            cmd=[-1]+h+z+y+v+[self.base_poten]+[self.effector_poten]+[-1]
            
            self.arduino_data(cmd)
            
            self.time_step+=0.1
            
        # If time step crosses 2pi then change the trot pair of legs 
        if self.time_step>=(2*math.pi):
            self.time_step=0
            self.phase_cnt+=1
            self.trot_flag=1-self.trot_flag
            self.motion_flag=1
        
        # One trot cycle lasts 3 phases [2,3]>>[1,4]>>[2,3]
        if self.phase_cnt>=3:
            self.phase_cnt=0


    # Function to convert the cmd list to byte stream and send it to arduino
    def arduino_data(self,cmd):
        self.ser.reset_input_buffer()
        
        data=struct.pack('20f',*cmd)
        
        self.ser.write(data) 

    # Function to set the bot in idle state, standing.
    def idle(self,delay=5):
        if self.hover: # Checking if the hover command is active
            cmd=[-1.0]+self.hover_ht+self.current_z+self.current_y+self.init_vel+[self.base_poten]+[self.effector_poten]+[-1.0]
        else:
            if self.start_balance: # Checks if pid balancer is active
                self.current_ht=self.pid.get_hts(self.roll,self.pitch,self.init_ht,[1,1,1,1])
            cmd=[-1.0]+self.current_ht+self.current_z+self.current_y+self.init_vel+[self.base_poten]+[self.effector_poten]+[-1.0]
            
        self.arduino_data(cmd)
        if delay!=0:
            time.sleep(delay)
        self.leg_moving=[0,0,0,0]

        self.time_step=0
        self.phase_cnt=0
        self.motion_flag=0
        self.trot_flag=1
    
    # Function to manually check if the legs are following IK commands properly
    def ik_test(self):
        leg_no=0
        while leg_no>=0:
            leg_no=int(input("Enter legno"))
            leg=[0,0,0,0]
            leg[leg_no]=1
            
            h=float(input("H val"))
            z=float(input("Z val"))
            y=float(input("Y val"))
            
            h_lst=[]
            z_lst=[]
            y_lst=[]
            for i in range(0,4):
                if i == leg_no:
                    h_lst.append(h)
                    z_lst.append(z)
                    y_lst.append(y)
                else:
                    h_lst.append(self.init_ht[0])
                    z_lst.append(self.init_z[0])
                    y_lst.append(self.init_y[0])
            cmd=[-1]+h_lst+z_lst+y_lst+[1000]*4+[self.base_poten]+[self.effector_poten]+[-1]
            self.arduino_data(cmd)
        
    
    
def main(args=None):
    rclpy.init(args=args)
    commander_node=Commander_Node()
    rclpy.spin(commander_node)
    commander_node.destroy_node()
    rclpy.shutdown()



if __name__=="__main__":
    main()


