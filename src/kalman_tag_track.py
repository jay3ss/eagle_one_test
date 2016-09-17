#!/usr/bin/env python
"""
QC Heads Up Display (HUD)
Displays sensor information from and commands sent to the QC
Created by: Josh Saunders
Date Created: 4/2/2016
Date Modified: 5/2/2016
"""
# Python libraries
from __future__ import print_function
# from filterpy.kalman import KalmanFilter
from Kalman import Kalman
import sys
import cv2
import math
import numpy as np


# We're using ROS here
import rospy
import roslib
from cv_bridge import CvBridge, CvBridgeError

# ROS messages
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata, navdata_altitude

class HUD:
    def __init__(self):
        self.image_pub = rospy.Publisher("heads_up",Image, queue_size=1000)

        self.bridge = CvBridge()
        # Subscribe to the correct topic
        self.image_sub = rospy.Subscriber("ardrone/image_raw",Image,self.cv_callback)
        self.navdata_sub = rospy.Subscriber("ardrone/navdata",Navdata,self.navdata_callback)

        self.tag_acquired = False
        self.tag_x = 0
        self.tag_y = 0
        self.tag_length = 0
        self.tag_width = 0
        self.tag_theta = 0

        self.x_pos = 0
        self.y_pos = 0

        # Kalman filter stuff
        #############
        # ATTEMPT 1 #
        #############
        # f = KalmanFilter (dim_x=2, dim_z=1)
        #
        # # Assign the initial value for the state (position and velocity)
        # f.x = np.array([[2.],    # position
        #                 [0.]])   # velocity
        #
        # # Define the state transition matrix
        # f.F = np.array([[1.,1.],
        #                 [0.,1.]])
        #
        # # Define the measurement function
        # f.H = np.array([[1.,0.]])
        #
        # # Define the covariance matrix
        # f.P *= 1000.
        #
        # # Assign the measurement noise
        # f.R = 5
        #############
        # ATTEMPT 2 #
        #############
        self.measurement = np.zeros((2,1)) # Measurement vector
        self.state = np.zeros((2,1))       # Initial state vector [x,y,vx,vy]
        self.kalman = Kalman()

    def cv_callback(self,data):
        """
        CAllback for the images streamed from the QC. Also, draws the HUD
        information.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        if self.tag_acquired:
            # Drawing some crosshairs
            self.crosshair(cv_image)

        cv2.imshow("QC HUD", cv_image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def navdata_callback(self,data):
        """
        Callback for the navdata subscriber.
        """
        if(data.tags_count > 0):
            self.tag_acquired = True

            # The positions need to be scaled due to the actual resolution
            # Actual resolution = 640 x 360
            # Data given as 1000 x 1000
            self.tag_x = int(data.tags_xc[0] * 640/1000)
            self.tag_y = int(data.tags_yc[0] * 360/1000)
            self.tag_theta = data.tags_orientation[0]
            self.tag_length = data.tags_height[0] * 360/1000
            self.tag_width = data.tags_width[0] * 640/1000
        else:
            self.tag_acquired = False

    def crosshair(self, cv_image):
        """
        Draws a crosshair over the center of the bounding of the tag
        """
        self.measurement[0, 0] = self.tag_x
        self.measurement[1, 0] = self.tag_y

        # Predict the position of the tag
        # f.predict()

        self.kalman.state_callback()
        self.kalman.measurement_callback(self.measurement)

        self.state = self.kalman.x

        self.x_pos = int(self.state[0, 0])
        self.y_pos = int(self.state[1, 0])
        
        # Draw the vertical line, then the horizontal, then the circle
        cv2.line(cv_image, (self.x_pos, self.y_pos + 25),(self.x_pos, self.y_pos - 25),(255,255,0),2)
        cv2.line(cv_image, (self.x_pos - 25, self.y_pos),(self.x_pos + 25, self.y_pos),(255,255,0),2)
        cv2.circle(cv_image, (self.x_pos, self.y_pos), 10, (255, 255, 0), 2)


def main(args):
    rospy.init_node('hud', anonymous=True)
    hud = HUD()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
