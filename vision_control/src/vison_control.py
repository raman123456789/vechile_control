#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import numpy as np

# zed custom messeges 
from zed_interfaces.msg import Object
from  zed_interfaces.msg import ObjectsStamped

import math
# custom scripts
# from depth_util import depth_check

class human_follow():

  def __init__(self):
    rospy.set_param('zed2/zed_node/object_detection/od_enabled',True)

    self.bridge = CvBridge()
    # subscribers for zed wrapper topics
    self.image_sub = rospy.Subscriber("/zed2/zed_node/rgb_raw/image_raw_color",Image,self.callback_image)
    self.object_sub = rospy.Subscriber("/zed2/zed_node/depth/depth_registered",Image,self.callback_depth)
    self.object_sub = rospy.Subscriber("/zed2/zed_node/obj_det/objects",ObjectsStamped,self.callback_objects)

    # publsuher for commad velocity
    self.cmd_vel_pub=rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=10)
    # initial varibles for 
    self.first_time=True
    self.twist=Twist()
    self.id=-1
    self.prev_human_obj_ids=[]
    self.l=0

    # twist messege list
    self.twist_velocity_list=[]
    
    # velocity parameters
    self.unit_velocity_step=0.2
    self.unit_steering_step_positive=0.1
    self.unit_steering_step_negative=-0.1

    # self.velocity_change_threshould=0.5
    self.skid_thesh=0.2

    self.i=0
    


    # distance set points
    self.obstacle_distance_stop_linear=1.5 # -- CHANGE
    self.obstacle_distance_stop_angular=0.5
    
    # human
    self.stop_distance=2.0  # -- CHANGE
    self.skid_distance_min=self.stop_distance#2.0
    self.skid_distance_max=2
    self.move_distance_min=3
    self.move_distance_max=10.0

    # obstacle 
    self.depth_check=False

    # depth image
    self.min_range=0.4
    self.max_range=16.0
    self.img_n=0


  def orange_pixel_count(self,img):
    
    ORANGE_MIN = np.array([5, 50, 50],np.uint8)
    ORANGE_MAX = np.array([15, 255, 255],np.uint8)
    cv2.imwrite('img'+str(self.img_n)+'.jpg',img)
    hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    frame_threshed = cv2.inRange(hsv_img, ORANGE_MIN, ORANGE_MAX)
    cv2.imwrite('img'+str(self.img_n)+'11'+'.jpg',frame_threshed)
    no_of_total_pixels=img.size
    total_orange_pixels=np.count_nonzero(frame_threshed)
    # self.img_n+=1
    print(total_orange_pixels)
    print(no_of_total_pixels)
    return float(total_orange_pixels)/float(no_of_total_pixels)



  def orange_predtictor(self,objects_data):
      oringe_pixel_list=[]
      label_ids_list=[]
      for obj in objects_data:
          if obj.label=='Person':
                a=obj.bounding_box_2d
                x1=a.corners[0].kp[0]
                y1=a.corners[0].kp[1]
                x2=a.corners[2].kp[0]
                y2=a.corners[2].kp[1]
                img=self.image[y1:y2,x1:x2]
                oringe_pixel_list.append(self.orange_pixel_count(img))
                label_ids_list.append(obj.label)

      self.prev_human_obj_ids=label_ids_list
      print(oringe_pixel_list)

      index=oringe_pixel_list.index(max(oringe_pixel_list))
      label_id=objects_data[index].label_id
      return label_id

     
    
  def twist_check(self,twist_msg):
        # self.l=len(self.twist_velocity_list)
        # result= False
        # if self.l>=4:
        #     if twist_msg==self.twist_velocity_list[self.l-2] and twist_msg==self.twist_velocity_list[self.l-3] and twist_msg==self.twist_velocity_list[self.l-4]:
        #         result=True
        #     else: 
        #         result= False
        # else:
        #     result= False

        # print(result)
        result=True
        return result



  def send_zeros_twist(self,x=0,z=0): #x,z are present it should look whether prev values are same or not if same then only we move. 
      if x==0 and z==0 :
          twist=Twist()
          self.twist_velocity_list.append(twist)
          self.cmd_vel_pub.publish(twist)

      else:
          
          
          twist=Twist()
          twist.linear.x=x
          twist.linear.y=self.target_x
          angle=self.target_y/self.target_x
          twist.linear.z=math.degrees(math.atan(angle))

          twist.angular.z=z
          self.twist_velocity_list.append(twist)
          check=self.twist_check(twist)
          if check:
            self.cmd_vel_pub.publish(twist)
          else:
              twist=Twist()
              self.cmd_vel_pub.publish(twist)


    
  # callback for left image of zed_camera
  def callback_image(self,data):

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.image=cv_image
    except CvBridgeError as e:
      print(e)
      self.image=[]


  def depth_valid(self,depth): 
    if self.min_range <= depth and depth <= self.max_range: # necessary condition
        return True
    elif  math.isinf(depth) and depth < 0: # Object too close to measure.
        return False
    elif  math.isinf(depth) and depth >0: # No objects detected in range.
        return False
    elif not math.isnan(depth): # This is an erroneous, invalid, or missing measurement.
        return False
    else:                        # The sensor reported these measurements as valid, but they are discarded per the limits defined by minimum_range and maximum_range.
        return False

  def callback_depth(self,depth_data):
    try:
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1") # depth_image = bridge.imgmsg_to_cv2(depth_data, "32FC1")
            # print(self.depth_image)
            
    except :
            print('Error in rendeing depth image')
            self.depth_check=False
            return 
 
    u = self.depth_image.shape[0]/2
    v = self.depth_image.shape[1]/2
    a=self.depth_image[v]
    self.obstale_pred=[]
    for i in a:
 
        if i < 1.0 :

            self.obstale_pred.append(0)
        
           
    print('len-------------------',len(self.obstale_pred))
    if len(self.obstale_pred) > 150:
        self.depth_check=False
        
    else:
        self.depth_check=True

    print('deth check-',self.depth_check)


  def callback_objects(self,data):
    if not self.depth_check:
        print(' Obstacle found -- stopped')
        self.twist.linear.x=0
        self.twist.angular.z=0
        self.twist_velocity_list.append(self.twist)
        self.twist_velocity_list
        self.cmd_vel_pub.publish(self.twist)
        return # UNCOMMENT IT 

    if len(data.objects)>0:
        ids_list=[data.objects[x].label_id  for x in range(len(data.objects)) ] 

        if self.id==-1:
            self.id=self.orange_predtictor(data.objects)
            self.send_zeros_twist()
            return 

        elif self.id not in ids_list:
            self.id=self.orange_predtictor(data.objects)
            self.send_zeros_twist()
            return
        elif self.id  in ids_list:

            orange=self.orange_predtictor(data.objects)

            if self.id==orange :


                print(" person id --- ", self.id )
                # command the vechile to move ahead
                for obj in (data.objects):
                    if obj.position[0] < self.obstacle_distance_stop_linear and   abs(obj.position[1]) <= self.obstacle_distance_stop_angular:
                        print('People close')
                        self.send_zeros_twist()
                        return
                index=ids_list.index(self.id)
                self.target_x=data.objects[index].position[0]
                self.target_y=data.objects[index].position[1]
                print('x, y --',self.target_x, self.target_y)

                if self.target_x   <=   self.stop_distance: # todo obstacle detection
                    print(" Target person is close to our vechile")
                    self.send_zeros_twist()
                    return
                elif self.target_x > self.skid_distance_min and self.target_x <= self.skid_distance_max: # skid mode
                    if self.target_y > 0 and self.target_y >= self.skid_thesh:
                        print("POS-SKID")
                        self.send_zeros_twist(0,self.unit_steering_step_positive)
                    
                    elif self.target_y < 0 and self.target_y <= - self.skid_thesh:
                        print('NEG-SKID')
                        self.send_zeros_twist(0,self.unit_steering_step_negative)
                    else:
                        print("ST-SKID")
                        self.send_zeros_twist()
                elif self.target_x  > self.move_distance_min and self.target_x <= self.move_distance_max:
                    if self.target_y > 0 and self.target_y >= self.skid_thesh:
                        print("POS-CHASE")
                        self.send_zeros_twist(self.unit_velocity_step,self.unit_steering_step_positive)
                    
                    elif self.target_y < 0 and self.target_y <= - self.skid_thesh:
                        print('NEG-CHASE')
                        self.send_zeros_twist(self.unit_velocity_step,self.unit_steering_step_negative)
                    else:
                        print("ST-CHASE")
                        self.send_zeros_twist(self.unit_velocity_step,0)

                elif self.target_x  > self.move_distance_max:
                    print("MAX-NO CHASSING")
            else:
                self.id=orange





def main(arg):
  rospy.init_node('Human_Follower', anonymous=True)
  ic = human_follow()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
