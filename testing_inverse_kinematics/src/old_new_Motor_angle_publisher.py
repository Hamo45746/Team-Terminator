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

state = 2

def store_colour(i):
    global colour
    colour = i.data


def store_state(i):
    global state
    state = i.data

def inverse_kinematics(pose: Pose) -> JointState:
    global pub_joint
    global colour
    global state
    print('received')
    L1 = 0.055
    L2 = 0.118
    L3 = 0.095
    L4 = 0.1
    robot_origin = [0 , 0 , 0]
    
    if state == 4: # intermediate state
        print('here')
        theta2 = 0.71
        theta3 = -2.2
        theta4 = 0.1
        theta1 = 0.00001

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

        pub_intermediate_state.publish(msg)

    elif state == 5:
        if colour == 'blue':
            theta1 = -2.6
        elif colour == 'red':
            theta1 = 2.6
        elif colour == 'green':
            theta1 = -1.25
        elif colour == 'yellow':
            theta1 = 1.25

        theta2 = 0.854
        theta3 = 1.16
        theta4 = -0.8

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

        pub_drop_state.publish(msg)

    #'''
    elif state == 2:
        desired_x = pose.position.x
        desired_y = pose.position.y
        desired_z = pose.position.z # elevation
        #desired_x = 0.15
        #desired_y = 0
        #desired_z = 0.1
        desired_end_angle = -90 * (np.pi/180)
        theta1 = np.arctan2(desired_y,desired_x)

        desired_distance = np.sqrt(desired_x**2 + desired_y**2)

        desired_distance_actual = desired_distance - L4*np.cos(desired_end_angle) #- np.sqrt(robot_origin[0]**2 + robot_origin[1]**2)
        desired_elevation_actual = desired_z - L4*np.sin(desired_end_angle) - L1

        #'''

        '''
        desired_x_actual = desired_x - L4*np.cos(desired_end_angle)*np.cos(theta1) - robot_origin[0]
        desired_y_actual = desired_y - L4*np.cos(desired_end_angle)*np.sin(theta1) - robot_origin[1]
        desired_z_actual = desired_z - L4*np.sin(desired_end_angle) - robot_origin[2]

        desired_distance_actual = np.sqrt(desired_x_actual**2 + desired_y_actual**2)
        desired_elevation_actual = desired_z_actual
        #'''

        # using lecture slides, treat as 2d problem
        plus_or_minus = -1
        costheta3 = (desired_distance_actual**2 + desired_elevation_actual**2 - L2**2 - L3**2) / (2*L2*L3)
        theta3 = m.atan2(plus_or_minus*m.sqrt(1 - costheta3**2),costheta3)
        theta2 = m.atan2(desired_elevation_actual,desired_distance_actual) - np.arctan2(L2*np.sin(theta3),L2 + L3*np.cos(theta3))

        theta2_abs = theta2
        theta3_abs = theta2 + theta3
        theta4_abs = desired_end_angle
        theta4 =  desired_end_angle - theta2 - theta3



        '''
        # FIND THE LEAST FACTOR ANGLE

        if theta1_abs > 0:
            theta1_abs = theta1_abs%(2*np.pi)
            if theta1_abs > np.pi:
                theta1_abs = -1*(2*np.pi - theta1_abs)
        else:
            theta1_abs = (1*theta1_abs)%(-2*np.pi)

        if theta2_relative > 0:
            theta2_relative = theta2_relative%(2*np.pi)
            if theta2_relative > np.pi:
                theta2_relative = -1*(2*np.pi - theta2_relative)
        else:
            theta2_relative = (theta2_relative)%(-2*np.pi)
            if theta2_relative < -np.pi:
                theta2_relative = 1*(2*np.pi - theta2_relative)

        if theta3_relative > 0:
            theta3_relative = (theta3_relative) % (2*np.pi)
            if theta3_relative > np.pi:
                theta3_relative = -1*(2*np.pi - theta3_relative)
        else:
            theta3_relative = (theta3_relative)%(-2*np.pi)
            if theta3_relative < -np.pi:
                theta3_relative = 1*(2*np.pi + theta3_relative)

        #'''

        # CORRECT THETA1 TO THE VERTICAL
        if theta2_abs > np.pi/2: # > 90deg
            theta2_abs = theta2_abs - np.pi/2
        elif theta2_abs < np.pi/2:
            theta2_abs = np.pi/2 - theta2_abs
        
        theta2=-theta2_abs
        theta4=-theta4


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



def main():
    global pub_intermediate_state,pub_joint,pub_drop_state
    # Create publisher
    pub_joint = rospy.Publisher(
        'desired_grab_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
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
        '   ', # Topic name
        Pose, # Message type
        inverse_kinematics # Callback function (required)
    )

    sub = rospy.Subscriber(
        'state', # Topic name
        msg.Int16, # Message type
        store_state # Callback function (required)
    )

    rospy.init_node('inverse_kinematics')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()


