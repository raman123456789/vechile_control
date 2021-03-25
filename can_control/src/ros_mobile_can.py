#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import can
import sys
from numpy import interp


baud_rate=rospy.get_param('baud_rate','250000')
channel_can=rospy.get_param('channel_can','vcan0')
print(channel_can)
bus_type_can=rospy.get_param('bustype_can','socketcan')
control=rospy.get_param('control','mobile')

try:
    bus = can.interface.Bus(bustype=bus_type_can, channel=channel_can, bitrate= baud_rate )

except Exception as e:
    print("---------------------------------------------------------------")
    print("                CONNECTION TO CAN DEVICE-- FAILED")
    print("     			PLEASE MAKE CAN UP              	 ")
    print("                                OR                                ")
    print("            PLEASE DO CHECK BAUDRATE AND CHANNEL NAME IN LAUNCH FILE")
    print(' ERROR is --',e)
    print("----------------------------------------------------------------")
    sys.exit()
   
print("---------------------------------------------------------------")

print("         CONNECTION TO CAN DEVICE-- SUCCESS")
print("---------------------------------------------------------------")
    
level_up=0.2
voltage_level=0
print(voltage_level)


class VECHILE_CONTROL():
    def __init__(self,min_voltage=0,max_voltage=5,step_voltage=0.2):
        self.min_voltage=min_voltage
        self.max_voltage=max_voltage
        self.step_voltage=step_voltage
        rospy.Subscriber("/turtle1/cmd_vel", Twist, self.callback)
        self.current_voltage=0
        


    def callback(self,data):
        x_linear=data.linear.x
        
        z_angular=data.angular.z


        if control=='mobile':
            b = interp(abs(x_linear), [0,1], [0,5])
            if abs(x_linear)>0 and self.current_voltage<self.max_voltage  and b>= self.current_voltage:
                self.current_voltage+=self.step_voltage 
            elif b < self.current_voltage:
                self.current_voltage-=self.step_voltage

            if x_linear==0:
                self.current_voltage=0

            print(self.current_voltage)
            voltage=int(self.current_voltage*256)
            hex_value=hex(voltage)[2:]
            print('len',len(hex_value))
         
            if len(hex_value)==4:
                d1=hex_value[:2]
                d2=hex_value[2:]
            elif len(hex_value)==3:
                d1='0'+hex_value[0]
                d2=hex_value[1:]
            elif len(hex_value)==2:
                d1='00'
                d2=hex_value
            elif len(hex_value)==1:
                d1='00'
                d2='0'+hex_value
            else:
                d1='00'
                d2='00'

            if x_linear>0:
                d3=0X1
            elif x_linear<0:
                d3=0X2
            else:
                d3=0X0

            print(d1,d2)
            d1 = int(d1, 16)
            d2 = int(d2, 16)

            msg=[d2,d1,d3,0X0,0X0,0X0,0X0,0X0]
            can_msg = can.Message(arbitration_id=0x510,
                              data=msg,
                              extended_id=False)
            print(msg)
            bus.send(can_msg)

            # steering code

            if z_angular==0:
                steer_angle=127
                
            elif z_angular>0:
                 steer_angle= interp(z_angular, [0,1], [127,212])
            
            elif z_angular<0:
                steer_angle = interp(z_angular, [-1,0], [38,127])
            else:
                pass
            print(steer_angle)

            hex_v=int(steer_angle)
            hex_v=hex(hex_v)
            angle_of_rotation = hex_v[2:]
            print(angle_of_rotation)
            angle_of_rotation = int(angle_of_rotation ,16) # angle to send to steering 
            mode=0X02 # angle control 
            

            steer_msg=[mode,angle_of_rotation,0x0,0X0,0X0,0X0,0X0,0X0]
            print(steer_msg)

            steer_can_msg = can.Message(arbitration_id=0x298,
                                    data=steer_msg,
                                    extended_id=False)
            bus.send(steer_can_msg)


        elif control=='vision':


            distance=data.linear.y # distace of person
            angle=data.linear.z # angle of person
            print(angle)
            steer_thesh=5
            voltage_step=0X2
            

            if x_linear==0:
                steer_msg=[0X0,0X0,0X0,0X0,0X0,0X0,0X0,0X0]
                steer_can_msg = can.Message(arbitration_id=0x298,
                                data=steer_msg,
                                extended_id=False)
                trotle_msg=[0X0,0X0,0X0,0X0,0X0,0X0,0X0,0X0]
                trotle_can_msg = can.Message(arbitration_id=0x510,
                                        data=trotle_msg,
                                        extended_id=False)
                bus.send(steer_can_msg)
                bus.send(trotle_can_msg)
            else: 
                if distance < 1:
                
                    angle_of_rotation=127+angle
                    angle_of_rotation=int(hex(int(angle_of_rotation))[2:],16) 
                    steer_msg=[0X02,angle_of_rotation,0X0,0X0,0X0,0X0,0X0,0X0]
                    steer_can_msg = can.Message(arbitration_id=0x298,
                                    data=steer_msg,
                                    extended_id=False)
                    trotle_msg=[0X0,0X0,0X0,0X0,0X0,0X0,0X0,0X0]
                    trotle_can_msg = can.Message(arbitration_id=0x510,
                                        data=trotle_msg,
                                        extended_id=False)

                    bus.send(steer_can_msg)
                    bus.send(trotle_can_msg)

                        

                                    
                                    
                elif distance >2 and distance<10:
                   
                        angle_of_rotation=127+angle
                        angle_of_rotation=int(hex(int(angle_of_rotation))[2:],16) 
                        steer_msg=[0X02,angle_of_rotation,0X0,0X0,0X0,0X0,0X0,0X0]
                        steer_can_msg = can.Message(arbitration_id=0x298,
                                        data=steer_msg,
                                        extended_id=False)
                        trotle_msg=[0X0,voltage_step,0X1,0X0,0X0,0X0,0X0,0X0]
                        trotle_can_msg = can.Message(arbitration_id=0x510,
                                        data=trotle_msg,
                                        extended_id=False)
                        bus.send(steer_can_msg)
                        bus.send(trotle_can_msg)
                        

                else:
                    steer_msg=[0X0,0X0,0X0,0X0,0X0,0X0,0X0,0X0]
                    steer_can_msg = can.Message(arbitration_id=0x298,
                                    data=steer_msg,
                                    extended_id=False)
                    trotle_msg=[0X0,voltage_step,0X0,0X0,0X0,0X0,0X0,0X0]
                    trotle_can_msg = can.Message(arbitration_id=0x510,
                                            data=trotle_msg,
                                            extended_id=False)
                    bus.send(steer_can_msg)
                    bus.send(trotle_can_msg)
                    



        elif control=='manual':
            pass
        else:
            print("invalid user control input")

        

      
    
def listener():
    rospy.init_node('can_control', anonymous=True)
    b=VECHILE_CONTROL(min_voltage=0,max_voltage=5,step_voltage=0.2)
    rospy.spin()

if __name__ == '__main__':
    listener()

