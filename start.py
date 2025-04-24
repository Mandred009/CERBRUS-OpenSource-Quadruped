#-- The script toe check if the trigger switch is on and then start the bot activation--#

import serial
import subprocess
import time
import sys

ser=serial.Serial("/dev/ttyUSB0", 9600)
time.sleep(2)

on_flag=0

while True:
            if ser.in_waiting>0:
                try:
                    data=ser.readline().decode()
                    data=data.split(',')
                    vals=data[0][1:-1].split("|")
                    if vals[-1]=='on' and on_flag==0:
                        subprocess.call("./nodes_start.sh")
                        ser.close()
                        sys.exit(0)
                        on_flag=1
                    time.sleep(0.1)
                except:
                    continue