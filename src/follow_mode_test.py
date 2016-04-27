#! /usr/bin/env python
"""
Follow Mode Test
Written by: Josh Saunders
Date: 4/11/2016

This is to test the PID that controls the 4 Degrees Of Freedom (DOF) of the QC
"""
# We're using ROS
import rospy

# Python libraries
import math

# The classes that we're using
from Controller import Controller
from Navdata import navdata_info

# The messages that we need
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

def is_in_box(minimum, maximum, position):
    """
    Checks if the position is within the given bounds
    """
    if ((minimum < position) and (position < maximum)):
        # print("In box")
        return True
    else:
        # print("Out box")
        return False
#
# def scale_theta(theta, theta_min):
#     """
#     Appropriately scales the given theta using the minimum value of theta. It
#     removes an offset in order to have the QC rotate counterclockwise
#     """
#     if((theta_min < theta) and (theta < 180)):
#         theta -= 360
#
#     # if (-178 < theta) and (theta < 178):
#     #     theta /= (theta - 180)
#     # else:
#     #     theta = default
#     return theta

def main():
    rospy.init_node('follow_mode_test')
    rate     = rospy.Rate(50) # 200Hz
    pub_ctrl = rospy.Publisher('cmd_vel', Twist, queue_size=100)

    qc      = Twist()
    navdata = navdata_info()
    ctrl    = Controller()

    ########################
    # Set the bounding box #
    ########################
    # X is in front and behind QC [0, 360] pixels
    # Y is left and right of QC   [0, 640] pixels
    bbx_max = 575
    bbx_min = 425
    bby_max = 575
    bby_min = 425
    yaw_max = 350
    yaw_min = 10

    ####################################
    # Setup the individual controllers #
    ####################################
    # Note: in order to change the values for integrator max and min, you need
    # to go into Controller.py and adjust the values there
    # Set yaw controller
    # NOTE: this doesn't seem to be working correctly...
    ctrl.pid_theta.setKp(1/260.0)
    # ctrl.pid_theta.setKp(0.0)
    ctrl.pid_theta.setKi(0.0)
    ctrl.pid_theta.setKd(0.0)
    ctrl.pid_theta.setPoint(180.0)
    default = 10
    # ctrl.pid_theta.setIntegrator(100)
    # ctrl.pid_theta.setDerivator(100)

    # Set the x (forward/backward) controller
    ctrl.pid_x.setKp(1/200.0)
    ctrl.pid_x.setKi(0)
    ctrl.pid_x.setKd(1/500.0)
    ctrl.pid_x.setPoint(500.0)
    ctrl.pid_x.setIntegrator(5000.0)
    ctrl.pid_x.setDerivator(5000.0)

    # Set the y (left/right) controller
    ctrl.pid_y.setKp(1/200.0)
    # ctrl.pid_y.setKp(0.0)
    ctrl.pid_y.setKi(0.0)
    ctrl.pid_y.setKd(0.0)
    ctrl.pid_y.setPoint(500.0)
    # ctrl.pid_y.setIntegrator(5000)
    # ctrl.pid_y.setDerivator(5000)

    # Set the z (altitude) controller
    ctrl.pid_z.setKp(0.0)
    ctrl.pid_z.setKi(0.0)
    ctrl.pid_z.setKd(0.0)
    ctrl.pid_z.setPoint(0.0)
    # ctrl.pid_z.setIntegrator(500)
    # ctrl.pid_z.setDerivator(500)

    # Disable hover mode
    qc.angular.x = 0.5
    qc.angular.y = 0.5

    # controller update values
    yaw_update = 0
    x_update   = 0
    y_update   = 0
    z_update   = 0

    # i = 0

    while not rospy.is_shutdown():
        # always update the altitude
        z_update = ctrl.pid_z.update(z_update)
        # print("Theta %.2f"  % navdata.theta)
        # print("(%d, %d)"  % (navdata.tag_x, navdata.tag_y))

        if navdata.tag_acquired:
            # print("Tag acquired %d" % i)
            # i += 1
            # If 10 < theta < 350 then let's rotate
            if ((yaw_min < navdata.theta) and (navdata.theta < yaw_max)):
                # We need to make sure that we have an offset so that the QC
                # rotates correctly
                # if((180 < navdata.theta) and (navdata.theta < yaw_max)):
                #     navdata.theta -= 360
                yaw_update  = ctrl.pid_theta.update(navdata.theta)
            else:
                yaw_update = 0

            # If the QC is in the bounding box then we should enter 'Hover'
            # mode and just hang there
            # is_in_box(minimum, maximum, position)
            if (is_in_box(bbx_min, bbx_max, navdata.tag_y) and is_in_box(bby_min, bby_max, navdata.tag_x)):
                x_update = 0
                y_update = 0
                qc.angular.x = 0.5
                qc.angular.y = 0.5
                # print("In the Box")
            # It's not in the bounding box therefore we should update the PIDs
            # and disable Hover mode
            else:
                x_update  = ctrl.pid_x.update(navdata.tag_x)
                y_update  = ctrl.pid_y.update(navdata.tag_y)
                qc.angular.x = 0.5
                qc.angular.y = 0.5
                # print("%.3f" % ctrl.pid_x.getError())

        # Make sure that we're not making any drastic updates
        # qc.angular.z = ctrl.pid_theta.avoid_drastic_corrections(yaw_update)
        # qc.linear.x  = ctrl.pid_x.avoid_drastic_corrections(x_update)
        # qc.linear.y  = ctrl.pid_y.avoid_drastic_corrections(y_update)
        # qc.linear.z  = ctrl.pid_z.avoid_drastic_corrections(z_update)

        qc.angular.z = yaw_update
        qc.linear.x  = x_update
        qc.linear.y  = y_update
        # qc.linear.z  = z_update

        pub_ctrl.publish(qc)
        rate.sleep()


if __name__=='__main__':
    main()
