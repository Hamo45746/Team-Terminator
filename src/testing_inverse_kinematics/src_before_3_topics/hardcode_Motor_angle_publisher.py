#!/usr/bin/python3


# Always need this
import rospy

# Import message types
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import std_msgs.msg as msg

# inverse kinematics stuff
import numpy as np
import math as m


print('motor publisher running')

# tf2 quat to euler
colour = 'test'


def store_colour(i):
    global colour
    colour = i.data


def store_state(i):
    global state
    state = i.data


state = 2
def inverse_kinematics(pose: Pose) -> JointState:
    global pub_joint,colour,state
    print('received')
    L1 = 0.055
    L2 = 0.118
    L3 = 0.094
    L4 = 0.1
    robot_origin = [0 , 0 , 0]
    print('\n'',state:',(state),'\n')
    if (state==1): # intermediate state
        print('IN PUBLISHER STATE 1')
        theta1 = 0
        theta2 = 0.72
        theta3 = -1.44
        theta4 = 0.112

        msg = JointState(
                # Set header with current time
                header=Header(stamp=rospy.Time.now()),
                # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
                name=['joint_1','joint_2', 'joint_3', 'joint_4']
            )

        msg.position = [
            theta1,
            theta2,
            theta3,
            theta4
            ]   
        i=0
        while (i < 50):
            pub_joint.publish(msg)
            i += 1

    if (state == 4): # intermediate state
        print('IN PUBLISHER STATE 1')
        theta1 = 0
        theta2 = -1
        theta3 = 0.35
        theta4 = -0.5

        msg = JointState(
                # Set header with current time
                header=Header(stamp=rospy.Time.now()),
                # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
                name=['joint_1','joint_2', 'joint_3', 'joint_4']
            )

        msg.position = [
            theta1,
            theta2,
            theta3,
            theta4
            ]   
        i=0
        while (i < 1):
            pub_joint.publish(msg)
            i += 1


    elif state == 5:
        #colour = 'B'
        if colour == "B":
            theta1 = -2.6
        elif colour == "R":
            theta1 = 2.6
        elif colour == "G":
            theta1 = -1.5
        elif colour == "Y":
            theta1 = 1.5
        else:
            theta1=0.1

        theta2 = -0.9
        theta3 = -0.93
        theta4 = 0.44

        msg = JointState(
                # Set header with current time
                header=Header(stamp=rospy.Time.now()),
                # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
                name=['joint_1','joint_2', 'joint_3', 'joint_4']
            )

        msg.position = [
            theta1,
            theta2,
            theta3,
            theta4
            ]   # THIS DOESNT INCLUDE HORIZONTAL  

        pub_joint.publish(msg)

    #'''
    elif state == 2:
        desired_x = pose.position.x
        desired_y = pose.position.y +0.015
        desired_z = pose.position.z # elevation

        
        deg_0_threshhold = 0.22
        deg_45_thresh = 0.16

        min_x = 0.08

        if desired_x < deg_45_thresh:
            desired_end_angle = -90 * (np.pi/180)
            desired_z=-0.05
        elif (deg_45_thresh <= desired_x <= deg_0_threshhold):
            desired_end_angle = -45 * (np.pi/180)
            desired_z=-0.03
        elif (desired_x > deg_0_threshhold):
            desired_end_angle = -5 * (np.pi/180)
            desired_z=0.0
        
        theta1 = np.arctan2(desired_y,desired_x)

        desired_distance = np.sqrt(desired_x**2 + desired_y**2)
        offset = 0.01
        desired_distance_actual = desired_distance - L4*np.cos(desired_end_angle) - offset*np.sin(desired_end_angle)
        desired_elevation_actual = desired_z - L4*np.sin(desired_end_angle) - L1 - offset*np.cos(desired_end_angle)  

        plus_or_minus = -1
        costheta3 = (desired_distance_actual**2 + desired_elevation_actual**2 - L2**2 - L3**2) / (2*L2*L3)
        theta3 = m.atan2(plus_or_minus*m.sqrt(1 - costheta3**2),costheta3)
        theta2 = m.atan2(desired_elevation_actual,desired_distance_actual) - np.arctan2(L2*np.sin(theta3),L2 + L3*np.cos(theta3))
        
        theta4 =  desired_end_angle - theta2 - theta3

        # CORRECT THETA2 TO THE VERTICAL
        if theta2 > np.pi/2: # > 90deg
            theta2 = theta2 - np.pi/2
        elif theta2 < np.pi/2:
            theta2 = np.pi/2 - theta2
        
        # Account for Dynamixel rotation directions
        theta2=-theta2
        theta4=-theta4

        msg = JointState(
                header=Header(stamp=rospy.Time.now()),
                name=['joint_1','joint_2', 'joint_3', 'joint_4']
            )

        msg.position = [
            theta1,
            theta2,
            theta3,
            theta4
            ]
        i=0
        while (i < 1):
            pub_joint.publish(msg)
            i+=1



def main():
    global pub_intermediate_state,pub_joint,pub_drop_state
    # Create publisher
    pub_joint = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=50 # Topic size (optional)
    )

    pub_intermediate_state = rospy.Publisher(
        'desired_intermediate_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )

    pub_drop_state = rospy.Publisher(
        'desired_drop_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )
  
    # Create subscriber
    sub = rospy.Subscriber(
        'desired_position', # Topic name
        Pose, # Message type
        inverse_kinematics # Callback function (required)
    )
    #'''
    sub = rospy.Subscriber(
        'state', # Topic name
        msg.Int16, # Message type
        store_state # Callback function (required)
    )

    sub = rospy.Subscriber(
        'colour', # Topic name
        msg.String, # Message type
        store_colour # Callback function (required)
    )

    rospy.init_node('inverse_kinematics')


    rospy.spin()


if __name__ == '__main__':
    main()