#!/usr/bin/python3
# Always need this
import rospy
import pigpio
# Import message types
import std_msgs.msg as msg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# inverse kinematics stuff
import numpy as np
import math as m


'''
GET GRIPPER TO LOOK AT STATE SO DOESNT OPEN WHEN MOVEING TO INTERMEDIATE ETC
'''


state = 2

#rpi = pigpio.pi()
#rpi.set_mode(18, pigpio.OUTPUT)

print('gripper running')


def store_state(i):
    global state
    state = i.data

'''
def publish_joint(i): 
    if state == (2):
        desired_actual = i
        pub.publish(desired_actual)
    elif (state == 4):
        desired_actual = i
        pub.publish(desired_actual)
    elif (state == 5):
        desired_actual = i
        pub.publish(desired_actual)
#'''

def publish_grab_joint(i):
    global state
    if state==2:
        pub.publish(i)

def publish_intermediate_joint(i):
    global state
    if state==4:
        pub.publish(i)

def publish_drop_joint(i):
    global state
    if state==5:
        pub.publish(i)


if __name__ == '__main__':
    global pub
    # Create publisher


    # Create subscriber
    sub = rospy.Subscriber(
        'state', # Topic name
        msg.Int16, # Message type
        store_state # Callback function (required)
    )

    sub = rospy.Subscriber(
        'desired_intermediate_states', # Topic name
        JointState, # Message type
        publish_intermediate_joint # Callback function (required)
    )

    sub = rospy.Subscriber(
        'desired_grab_states', # Topic name
        JointState, # Message type
        publish_grab_joint # Callback function (required)
    )

    sub = rospy.Subscriber(
        'desired_drop_states', # Topic name
        JointState, # Message type
        publish_drop_joint # Callback function (required)
    )

    pub = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )

    # Initialise node with any node name
    rospy.init_node('joint_converter')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()

'''
if __name__ == '__main__':
    main()
#'''










