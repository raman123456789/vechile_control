#!/usr/bin/env python
import os
import sys
import can



bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=250000)
msg1=can.Message(arbitration_id=0x501, data=[101,0,0],is_extended_id=False)
msg3=can.Message(arbitration_id=0x501, data=[101,0,0x02],is_extended_id=False)
msg4=can.Message(arbitration_id=0x501, data=[101,0,0x05],is_extended_id=False)

autoModeRequestAccepted = False
autoMode = False


def start():
    while True:
        xa=input("Do you want to neter auto mode? 'y'") 
        if (xa=="y"):
            bus.send(msg1)
            while True:
                message=bus.recv()
                msg=message.data
                if message.arbitration_id==0x501:
                    if msg[2]==0x01:
                        bus.send(msg3)
                        autoModeRequestAccepted = True

                    elif msg[2]==0x04 and autoModeRequestAccepted:
                        autoModeRequestAccepted = False
                        autoMode= True
                        print ("we are in autonomus mode")
                        print("close stop auto mode by presing ctrl+c")
                        break
                    elif msg[2]==0x03:
                        break
            
            while autoMode:
                message=bus.recv()
                msg=message.data
                if message.arbitration_id==0x501:
                    if msg[2]==0x03:
                        autoMode = False
                        print("Automode interrupted")
                        if msg[1]==10:
                            print("Drive command message timeout")
                        elif msg[1]==11:
                            print("no response from on-board computer")
                        elif msg[1]==12:
                            print("user pressed throttle")
                        elif msg[1]==13:
                            print("user changed FNR state")
                        
                    elif msg[2]==0x04 and msg[1]==0x01:
                        bus.send(msg4)
start()

# def stop():
#     while True:
        
#         a = input("")
#         if a == "n":
#             sys.exit(start)



                


            
   