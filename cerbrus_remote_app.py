#-- The GUI script to run on the contoller for monitoring the telemetry--#
#-1. Better not mess with this script unless you know what you are doing :)

import customtkinter
import serial
import threading
import sys
from PIL import Image


customtkinter.set_appearance_mode("dark")  # Modes: system (default), light, dark
customtkinter.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green
image = Image.open("mandred_icon2.png")
background_image = customtkinter.CTkImage(image, size=(300, 300))


class FullscreenApp:
    def __init__(self, master):
        self.ser=serial.Serial(port="/dev/ttyUSB0",baudrate=9600)
        self.telemetry_no=8 # imp change this no if you change the no of telemetry data points
        self.data_vals=[0]*self.telemetry_no
        
        self.master = master
        
        self.master.attributes("-fullscreen", "True")  # Set fullscreen
        self.master.geometry("476x320")  # Set window size
        
        bg_lbl = customtkinter.CTkLabel(self.master, text="", image=background_image)
        bg_lbl.place(relx=0.5, rely=0.5, anchor=customtkinter.CENTER)
        
        # Create and place the Exit button
        self.exit_button = customtkinter.CTkButton(
            master=self.master, 
            text="Exit", 
            command=self.exit_button_function,
            fg_color="red",
            corner_radius=50,  # Adjust the corner radius to change the button shape
            width=120,  # Adjust the width
            height=30   # Adjust the height
        )
        self.exit_button.place(relx=0.5, rely=0.95, anchor=customtkinter.CENTER)
        
        self.title_label = customtkinter.CTkLabel(
            master=self.master, 
            text="C E R B R U S < | > R E M O T E",  # Initial text
            font=("Georgia", 18,"bold"),
            text_color="white"
        )
        self.title_label.place(relx=0.5, rely=0.05, anchor=customtkinter.CENTER)
        
        self.pitch_label = customtkinter.CTkLabel(
            master=self.master, 
            text="Pitch",  # Initial text
            font=("Times New Roman", 15,"bold"),
            text_color="white"
        )
        self.pitch_label.place(relx=0.1, rely=0.35, anchor=customtkinter.CENTER)
        
        self.canvas_pitch = customtkinter.CTkCanvas(master=self.master, width=70, height=70, bg="black")
        self.canvas_pitch.place(relx=0.1, rely=0.2, anchor=customtkinter.CENTER)
        
        self.pitch_val = self.canvas_pitch.create_text(35, 35, text="360", font=("Arial", 16, "bold"), fill="orange")
        
        
        self.roll_label = customtkinter.CTkLabel(
            master=self.master, 
            text="Roll",  # Initial text
            font=("Times New Roman", 15,"bold"),
            text_color="white"
        )
        self.roll_label.place(relx=0.3, rely=0.35, anchor=customtkinter.CENTER)
        
        self.canvas_roll = customtkinter.CTkCanvas(master=self.master, width=70, height=70, bg="black")
        self.canvas_roll.place(relx=0.3, rely=0.2, anchor=customtkinter.CENTER)
        
        self.roll_val = self.canvas_roll.create_text(35, 35, text="360", font=("Arial", 16, "bold"), fill="orange")
        
        self.yaw_label = customtkinter.CTkLabel(
            master=self.master, 
            text="Yaw",  # Initial text
            font=("Times New Roman", 15,"bold"),
            text_color="white"
        )
        self.yaw_label.place(relx=0.5, rely=0.35, anchor=customtkinter.CENTER)
        
        self.canvas_yaw = customtkinter.CTkCanvas(master=self.master, width=70, height=70, bg="black")
        self.canvas_yaw.place(relx=0.5, rely=0.2, anchor=customtkinter.CENTER)
        
        self.yaw_val = self.canvas_yaw.create_text(35, 35, text="360", font=("Arial", 16, "bold"), fill="orange")

        self.balancer_label = customtkinter.CTkLabel(
            master=self.master, 
            text="Balancer",  # Initial text
            font=("Times New Roman", 15,"bold"),
            text_color="white"
        )
        self.balancer_label.place(relx=0.7, rely=0.35, anchor=customtkinter.CENTER)
        
        self.canvas_balancer = customtkinter.CTkCanvas(master=self.master, width=70, height=70, bg="black")
        self.canvas_balancer.place(relx=0.7, rely=0.2, anchor=customtkinter.CENTER)
        
        self.balancer_circle = self.canvas_balancer.create_oval(
            10, 10, 60, 60,  # Define the bounding box (x1, y1, x2, y2)
            fill="red",  # Initial color (False state)
        )
        
        
        self.battery_progress = customtkinter.CTkProgressBar(
            master=self.master,
            orientation="vertical",  # Set to vertical
            width=20,
            height=150,  # Height of the progress bar
            progress_color="yellow",  # Color of the progress bar
            mode="determinate"  # Use determinate mode for a fixed value
        )
        self.battery_progress.place(relx=0.9, rely=0.4, anchor=customtkinter.CENTER)
        
        self.battery_value_label = customtkinter.CTkLabel(
            master=self.master,
            text=f"{12} V",  # Display battery voltage as a percentage
            font=("Arial", 12),
            text_color="white"
        )
        self.battery_value_label.place(relx=0.9, rely=0.7, anchor=customtkinter.CENTER)
        
        self.auto_label = customtkinter.CTkLabel(
            master=self.master, 
            text="Auto Mode",  # Initial text
            font=("Times New Roman", 15,"bold"),
            text_color="white"
        )
        self.auto_label.place(relx=0.1, rely=0.65, anchor=customtkinter.CENTER)
        
        self.canvas_auto = customtkinter.CTkCanvas(master=self.master, width=70, height=70, bg="black")
        self.canvas_auto.place(relx=0.1, rely=0.5, anchor=customtkinter.CENTER)
        
        self.auto_circle = self.canvas_auto.create_oval(
            10, 10, 60, 60,  # Define the bounding box (x1, y1, x2, y2)
            fill="red",  # Initial color (False state)
        )

        self.wifi_name_label = customtkinter.CTkLabel(
            master=self.master, 
            text="Wi-Fi Name:",  # Initial text
            font=("Times New Roman", 15,"bold"),
            text_color="yellow"
        )
        self.wifi_name_label.place(relx=0.2, rely=0.74, anchor=customtkinter.CENTER)
        
        self.wifi_ip_label = customtkinter.CTkLabel(
            master=self.master, 
            text="IP:",  # Initial text
            font=("Times New Roman", 15,"bold"),
            text_color="yellow"
        )
        self.wifi_ip_label.place(relx=0.2, rely=0.82, anchor=customtkinter.CENTER)

        self.get_data()
    
    def change_vals(self):
    # Use after to update the UI on the main thread
        self.master.after(0, self.update_vals)

    def update_vals(self):
        self.wifi_name_label.configure(text=f"Wi-Fi Name: {self.data_vals[0].strip()}")
        self.wifi_ip_label.configure(text=f"IP: {self.data_vals[1].strip()}")
        
        self.canvas_pitch.itemconfig(self.pitch_val, text=self.data_vals[2])
        self.canvas_roll.itemconfig(self.roll_val, text=self.data_vals[3])
        self.canvas_yaw.itemconfig(self.yaw_val, text=self.data_vals[4])
        
        if self.data_vals[5]=="True":
            self.canvas_balancer.itemconfig(self.balancer_circle, fill="green")  # Color for True
        else:
            self.canvas_balancer.itemconfig(self.balancer_circle, fill="red")  # Color for False
            
            
        battery_percent=round(((float(self.data_vals[6])-10)/2),2) # considering safe lower voltage as 10 and upper as 12
        self.battery_value_label.configure(text=f"{self.data_vals[6].strip()} V")
        self.battery_progress.set(battery_percent)  # Set progress bar value
        
        if battery_percent > 0.7:
            self.battery_progress.configure(progress_color="green")  # Green for > 70%
        elif battery_percent > 0.3:
            self.battery_progress.configure(progress_color="yellow")  # Yellow for 30% to 70%
        else:
            self.battery_progress.configure(progress_color="red")  # Red for < 30%
        
        
        if self.data_vals[7]=="1":
            self.canvas_auto.itemconfig(self.auto_circle, fill="green")  # Color for True
        else:
            self.canvas_auto.itemconfig(self.auto_circle, fill="red")  # Color for False
        
        
    def exit_button_function(self):
        sys.exit()  # Exit the application
        
    def get_data(self):
        def read_serial():
            while True:
                if self.ser.in_waiting > 0:
                    data = self.ser.readline().decode('utf-8')

                    temp = data.split(",")
                    
                    if len(temp) == self.telemetry_no:
                        self.data_vals = temp
                        self.change_vals()

    # Start the reading thread
        threading.Thread(target=read_serial, daemon=True).start()
        
        

# Initialize the main window and run the application
if __name__ == "__main__":
    app = customtkinter.CTk()  # Create CTk window
    FullscreenApp(app)  # Initialize the FullscreenApp class with the CTk window
    app.mainloop()  # Run the application's main loop




