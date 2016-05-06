#! /usr/bin/env python
"""
This is where everything is going to live
"""

# We're using ROS here
import rospy

# ROS message
from std_msgs.msg import String, Empty

# Import all of the modes
from Land          import Landing
# from Follow        import Follow
from Takeoff       import Takeoff
from Emergency     import Emergency
from TakePicture   import TakePicture
from Reacquisition import Reacquisition

rospy.init_node('main')
rate = rospy.Rate(10)

state = 'nada'
transition = 'nada'

def state_cb(msg):
    state = msg.data

pub_transition = rospy.Publisher('/smach/transition', String, queue_size=1)
sub_state = rospy.Subscriber('/smach/state', String, state_cb)

# Instantiate all of the modes
# Set all of the necessary parameters
# TODO instantiate these correctly i.e. correct parameters
# Land Mode Instantiation
land_speed = -1	 # m/s
land_min_alt = 1000  # mm
land_max_alt = 2000 # mm
land_height_diff = 0 #mm
land_timeout = 3 # seconds
land = Landing(land_speed, land_min_alt, land_height_diff, land_max_alt, land_timeout)

# Takeoff Mode Instantiation
takeoff_speed = 1 # percentage of max speed (kind of)
takeoff_max_alt = 2500  # mm
takeoff_timeout = 10 # seconds
takeoff_counter = 0 # times
takeoff = Takeoff(takeoff_speed, takeoff_max_alt, takeoff_timeout)

# Takeoff Mode Instantiation
emergency = Emergency()

# Take Picture Mode Instantiation
takepicture_time = 30 # seconds
# takepicture_max_time   = 3 # seconds
takepicture = TakePicture(takepicture_time)

# Reacquisition Mode Instantiation
reac_velocities = (0.3, 0.3, 0.3) 	 # m/s
reac_max_alt = 3000  # mm
reac_tag_timeout = 15 	 # seconds
reac_prev_state_timeout = 5 # seconds
reac = Reacquisition(reac_velocities, reac_max_alt, reac_tag_timeout, reac_prev_state_timeout)
# follow = Follow()

# The start of it all
def main():
    print state

    # NOTE: There are some subtleties here. We want to be able to use more than
    # one mode for certain modes which is why follow mode is on it's own
    # Check if we're in takeoff mode
    if (state == 'takeoff'):
        print "Takeoff Mode"
        # NOTE: This was copy and pasted from the takeoff node
        # TODO: Test all of this
        # To get this guy to take off! For some reason just calling this
        # function once does not work. This value (50) was determined
        # experimentally
        while takeoff_counter < 50:
            takeoff.launch()
            takeoff_counter += 1
            rate.sleep()
        if(takeoff.altitude < takeoff.max_altitude):
            rospy.loginfo("Go up!")
            takeoff.change_altitude(takeoff_speed)
        elif(takeoff.altitude >= takeoff.max_altitude):
            takeoff_speed = 0
            rospy.loginfo("Stop!")
            # takeoff.change_altitude(takeoff_speed)
            # To change states, we publish the fact that we've
            # reached our takeoff altitude
            rospy.loginfo("Going to follow mode")
            takeoff.goto_follow()

    # Check if we're in land mode
    elif (state == 'land'):
        height_diff = land.max_altitudeGoal - land.altitude
        print("%d" % land.altitude)
            # if(land.tag_acquired):
        if(land.altitude > land.min_altitude):
            print("Go down")
            if(land.height_diff > land.min_altitude):
                land.change_altitude(land_speed)
                print("Descending")
                # TODO: Check this. Is it nececessary? Seems odd.
                rospy.sleep(.5)
            # TODO: Check this. Is it nececessary? Seems odd.
            elif(land.height_diff <= land.min_altitude):
                pass
        elif(land.altitude < land.min_altitude):
            print("Eagle one has descended!")
            land.land()

    # Check if we're in take picture mode
    elif (state == 'take_picture'):
        print "Take Picture Mode"
        if(takepicture.finished()):
            print "Picture taken"
    # Check if we're in emergency mode
    elif (state == 'emergency'):
        print "Emergency Mode"
        emergency.emergency_land()

    # Check if we're in reacquisition mode
    elif (state == 'reacquisition'):
        print "Reacquisition Mode"
        reac.move()

    # Check if we're in secure mode and do nothing
    elif (state == 'secure'):
        # Do nothing
        print "Secure Mode"

    # Check if we're in follow mode
    # Follow mode is used in every mode except for secure and emergency
    if (not (state == 'secure') or not (state == 'emergency')):
        print "Follow Mode"

    rospy.spin()

if __name__ == '__main__':
    main()
