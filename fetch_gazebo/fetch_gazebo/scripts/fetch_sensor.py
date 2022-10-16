from dataclasses import dataclass
import posix
from tkinter import Y
import cv2
import sys
import os

from matplotlib.backend_bases import LocationEvent
import roslib
import time
import rospy
import numpy as np
import math

import geometry_msgs.msg 
import tf #transform_data ypes

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64MultiArray, Float32
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped #, PoseWithConvariance, TwistWithConvariance
from gazebo_msgs.msg import ModelStates
from fetch_robot_sim.msg import RGB_Image_Info
from fetch_robot_sim.msg import Location_3D

from std_msgs.msg import String
# Create an object to read camera video
class RGBD_Detection:
    """
    This Class uses the header RGB-D camera to detect the object and visual servoing
    This Class also communicates with the GUI in order send information across. This information is sent via publishing to ROS
    
    Custom message files 'Location_3D' and 'RGB_Image_Info' are used 
    Can be used to detect colours and object
    
    Function cameraRGBCallBack searches for the Red, Green or Blue objects using the camera given with the fetch robot.
    This function deals with the visual servoing and draws rectangle using the points x,y,w,h. Using these data middle points are calculated and sent to cameraDepthCallback
    
    Function cameraDepthCallBack recieves x,y (in pixels) coordinates of the middle point of the object it is targeting from cameraRGBCallBack.
    As depth camera requests the x,y coordinates (from top left corner to bottom right corner) similar to RGB camera.
    It sends x y and z in m. x being perpendicular to the robot, y egin the height and z being the depth
    
    Funtion calculationsCallBack recieves the data from the cameraDepthCallBack and transfroms them into global coordinate frame.
    x is the ddepth, y is perpendicular and z is the hight.
    """
    def __init__(self):
        self.bridge_object = CvBridge()
        
        self.sync = 0 #when sync = 0 it runs RGB, sync = 1 Depth
    
        #Subscribers
        self.image_sub = rospy.Subscriber("/head_camera/rgb/image_raw", Image, self.cameraRGBCallBack)        
        self.depth_sub = rospy.Subscriber("/head_camera/depth_registered/image_raw",Image,self.cameraDepthCallBack)
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.calculationsCallBack)
        

        self.obj_detect_request = rospy.Subscriber("/object_detect_request",String,self.objRequest)
        self.obj_find = "Small Cube"
        #Publishers
        self.detect_object = rospy.Publisher("/object_info", RGB_Image_Info, queue_size=10)
        self.location_3D = rospy.Publisher("/distance",Location_3D, queue_size=10)
        self.obj_location_3D = rospy.Publisher("/obj_distance",Location_3D, queue_size=10)
        self.ub = rospy.Subscriber("/gazebo/model_states",ModelStates,self.obj_info)
        self.bounding_image = rospy.Publisher("/bounding_image",Image, queue_size=10)
        # self.bounding_image1 = rospy.Publisher("/bounding_image1",Image, queue_size=10)
        
        #Globals for the class
        self.midPoints = Location_3D(0, 0, 0)
        self.locationPos = Location_3D()
        self.posLocalGlobal = Location_3D()
        self.x = 0
        self.y = 0
        self.z = 0

    def objRequest(self,data):
        self.obj_find = str(data.data)
        # print(self.obj_find)
    

    def cameraRGBCallBack(self, data):
        # print("callback looping")
        cap = self.bridge_object.imgmsg_to_cv2(data, "bgr8")
        if self.sync == 0:
                        
            # Convert BGR to HSV
            hsv = cv2.cvtColor(cap, cv2.COLOR_BGR2HSV)
            # define blue colour range
            light_blue = np.array([102, 50, 0], np.uint8)
            dark_blue = np.array([255, 140, 140], np.uint8)

            # Threshold the HSV image to get only blue colours
            blue_mask = cv2.inRange(hsv, light_blue, dark_blue)

            # define red colour range
            light_red = np.array([0, 100, 20], np.uint8)
            dark_red = np.array([10, 255, 255], np.uint8)

            # Threshold the HSV image to get only red colours
            red_mask = cv2.inRange(hsv, light_red, dark_red)

            # define green colour range
            light_green = np.array([0, 100, 0], np.uint8)
            dark_green = np.array([80, 140, 80], np.uint8)

            # Threshold the HSV image to get only green colours
            green_mask = cv2.inRange(cap, light_green, dark_green)

            kernal = np.ones((5, 5), "uint8")
            blue_mask = cv2.dilate(blue_mask, kernal)
            red_mask = cv2.dilate(red_mask, kernal)
            green_mask = cv2.dilate(green_mask, kernal)

            # Bitwise-AND mask and original image
            output_blue = cv2.bitwise_and(cap, cap, mask=blue_mask)
            output_red = cv2.bitwise_and(cap, cap, mask=red_mask)
            output_green = cv2.bitwise_and(cap, cap, mask=green_mask)


            # Find the things 
            obj_info_msg = RGB_Image_Info()
            if(self.obj_find == str("Red Cylinder"))or (self.obj_find == str("All")):
                # Creating contour to track red colour for cylinder
                contours, hierarchy = cv2.findContours(
                    red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
                )

                #Drawing the rectangle
                for pic, contour in enumerate(contours):
                    area = cv2.contourArea(contour)
                    if area > 300:
                        x, y, w, h = cv2.boundingRect(contour)
                        cap = cv2.rectangle(cap, (x, y), (x + w, y + h), (0, 0, 255), 2)


                        # Preparing center point
                        self.midPoints.x = x + (w / 2)
                        self.midPoints.y = y + (h / 2)
                        obj_info_msg.x = self.midPoints.x
                        obj_info_msg.y = self.midPoints.y
                        obj_info_msg.objectName = "Red Cylinder"
                        #Text on the rectangle
                        cv2.putText(
                            cap,
                            "Red Cylinder",
                            (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0,
                            (0, 0, 255),
                        )
            if(self.obj_find == str("Green Cube")) or (self.obj_find == str("All")):
                # print("Finding large cube")
                # Creating contour to track green colour for large cube
                contours, hierarchy = cv2.findContours(
                    green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
                )
                #Drawing the rectangle
                for pic, contour in enumerate(contours):
                    area = cv2.contourArea(contour)
                    if area > 300:
                        x, y, w, h = cv2.boundingRect(contour)
                        cap = cv2.rectangle(cap, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        # Preparing center point
                        self.midPoints.x = x + (w / 2)
                        self.midPoints.y = y + (h / 2)
                        obj_info_msg.x = self.midPoints.x
                        obj_info_msg.y = self.midPoints.y
                        obj_info_msg.objectName = "Green Cube"
                        #Text on the rectangle
                        cv2.putText(
                            cap,
                            "Green Cube",
                            (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0,
                            (0, 255, 0),
                        )
            if(self.obj_find == str("Blue Cube"))or (self.obj_find == str("All")):
                # Creating contour to track blue colour for blue cube
                contours, hierarchy = cv2.findContours(
                    blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
                )
                #Drawing the rectangle
                for pic, contour in enumerate(contours):
                    area = cv2.contourArea(contour)
                    if area > 300:
                        x, y, w, h = cv2.boundingRect(contour)
                        cap = cv2.rectangle(cap, (x, y), (x + w, y + h), (255, 0, 0), 2)

                        # Preparing center point
                        self.midPoints.x = x + (w / 2)
                        self.midPoints.y = y + (h / 2)                        
                        obj_info_msg.x = self.midPoints.x
                        obj_info_msg.y = self.midPoints.y
                        obj_info_msg.objectName = "Blue Cube"
                        
                        #Text on the rectangle
                        cv2.putText(
                            cap,
                            "Blue Cube",
                            (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0,
                            (255, 0, 0),
                        )
            self.sync = 1
            # flip image as well
            self.bounding_image.publish(self.bridge_object.cv2_to_imgmsg(cv2.flip(cap, 1), "bgr8"))
            self.detect_object.publish(obj_info_msg)
            
            
    def cameraDepthCallBack(self,data):
        if self.sync == 1:
            cv_cap2 = self.bridge_object.imgmsg_to_cv2(data,"passthrough")
            #to ensure that there is a center point
            if self.midPoints.x > 0:
                if self.midPoints.y > 0:
                    #midpoints from RGB
                    x = int(self.midPoints.x)
                    y = int(self.midPoints.y)
                    
                    #depth when given x,y from camer's point of view
                    try:
                        self.z = cv_cap2[x,y]
                    except:
                        self.z = 0

                    #publishing result for IKsolver
                    
                    self.locationPos.x = x * 0.000265 #in m
                    self.locationPos.y = y * 0.000265 #in m
                    self.locationPos.z = self.z
                    #self.location_3D.publish(locationPos)
                    self.sync = 2
                else:
                    self.sync = 0 
            else:
                self.sync = 0
    def obj_info(self,data):
        pos = Location_3D()
        i = 0
        for item in data.name:
            if((str(item) == "blue_cube") and (self.obj_find == str("Blue Cube"))):
                pos.x = data.pose[i].position.x
                pos.y = data.pose[i].position.y
                pos.z = data.pose[i].position.z
            if((str(item) == "green_cube") and (self.obj_find == str("Green Cube"))):
                pos.x = data.pose[i].position.x
                pos.y = data.pose[i].position.y
                pos.z = data.pose[i].position.z
            if((str(item) == "cylinder") and (self.obj_find == str("Red Cylinder"))):
                pos.x = data.pose[i].position.x
                pos.y = data.pose[i].position.y
                pos.z = data.pose[i].position.z
            i += 1
        self.location_3D.publish(pos)



    def calculationsCallBack(self,data):
        if self.sync == 2:
            robotPos = Location_3D()
            robotPos.x = data.pose.pose.position.x
            robotPos.y = data.pose.pose.position.y
            robotPos.z = data.pose.pose.position.z  #position of the starting arm location?
            
            #quart1 = data.pose.orientation    #in quarternian
            rotation = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)  
            
            #transform quarterian to rpy, only yaw is required 
            euler = tf.transformations.euler_from_quaternion(rotation)
            roll = euler[0] # in rad
            pitch = euler[1] # in rad 
            yaw = euler[2] # in rad
            
            #print(robotPos)
            #print(yaw)
            
            self.posLocalGlobal.x = self.locationPos.z + 0.11  #z shows the depth
            if self.locationPos.z > 0:
                self.posLocalGlobal.y = ((self.locationPos.x / self.posLocalGlobal.x)*1.09) #x is width
                self.posLocalGlobal.z = ((self.locationPos.y / self.posLocalGlobal.x)*1.09) + 0.64#y is the hight
            self.obj_location_3D.publish(self.posLocalGlobal)
            # print(self.posLocalGlobal)
            self.sync = 0

        
        

if __name__ == "__main__":
    rospy.init_node("Paul_Python_Sensor")
    cv_thing = RGBD_Detection()

    while not rospy.is_shutdown():
        rospy.spin()


# work out transformation math

# find 0,0,0 of robot gripper position
# 

# Camera frame -> global frame (function of focal length, x,y,z)
# Global frame -> robot frame (x,y,z) + (rpy) (custom message)
# 
