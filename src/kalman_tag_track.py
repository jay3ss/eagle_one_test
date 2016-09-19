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
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata, navdata_altitude

class HUD:
    def __init__(self):
        self.image_pub = rospy.Publisher("tag_pose",Image, queue_size=1000)

        self.bridge = CvBridge()
        # Subscribe to the correct topic
        self.image_sub = rospy.Subscriber("ardrone/image_raw",Image,self.cv_callback)
        self.tag_pose_sub = rospy.Subscriber("filtered_tag_pose",Pose2D,self.tag_pose_cb)

        self.pose = Pose2D()

    def cv_callback(self,data):
        """
        CAllback for the images streamed from the QC. Also, draws the HUD
        information.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.crosshair(cv_image)

        cv2.imshow("QC HUD", cv_image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def tag_pose_cb(self,data):
        """
        Callback for the tag pose subscriber.
        """
        self.pose = data


    def crosshair(self, cv_image):
        """
        Draws a crosshair over the center of the bounding of the tag
        """
        tag_x = int(self.pose.x)
        tag_y = int(self.pose.y)

        # Draw the vertical line, then the horizontal, then the circle
        cv2.line(cv_image, (tag_x, tag_y + 25),(tag_x, tag_y - 25),(255,255,0),2)
        cv2.line(cv_image, (tag_x - 25, tag_y),(tag_x + 25, tag_y),(255,255,0),2)
        cv2.circle(cv_image, (tag_x, tag_y), 10, (255, 255, 0), 2)


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
