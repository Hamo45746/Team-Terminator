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
loop_num = 0
def inverse_kinematics(pose: Pose) -> JointState:
    global pub_joint,colour,state,loop_num
    print('received')
    L1 = 0.055
    L2 = 0.118
    L3 = 0.094
    L4 = 0.11
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
        '''
    elif (state==4): # intermediate state
        print('IN PUBLISHER STATE 1')
        theta1 = 0
        theta2 = -0.818
        theta3 = 0
        theta4 = -0.639783

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
        #'''

    elif state == 5:
        colour = 'B'
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
    
    elif (state == 2) or (state==4):
        print('hello')
        desired_x = pose.position.x
        desired_y = pose.position.y
        desired_z = pose.position.z # elevation
        
        #desired_x = 0.15
        #desired_y = 0
        #desired_z = 0.1

        if (state ==2) or (state==3):
            desired_z=0.0
            desired_end_angle = -45 * (np.pi/180)
        elif state==4:
            desired_end_angle = 45 * (np.pi/180)

        theta1 = np.arctan2(desired_y,desired_x)

        desired_distance = np.sqrt(desired_x**2 + desired_y**2)
        offset = 0.015
        desired_distance_actual = desired_distance - L4*np.cos(desired_end_angle) - offset*np.sin(desired_end_angle)  #- np.sqrt(robot_origin[0]**2 + robot_origin[1]**2)
        desired_elevation_actual = desired_z - L4*np.sin(desired_end_angle) - L1 - offset*np.cos(desired_end_angle)  

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
        
        if (state==4) and (loop_num == 5):
            desired_end_angle = -45 * (np.pi/180)
            loop_num += 1
        theta4 =  desired_end_angle - theta2 - theta3


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
            theta1*1,
            theta2,
            theta3,
            theta4
            ]   # THIS DOESNT INCLUDE HORIZONTAL  
        i=0
        while (i < 30):
            pub_joint.publish(msg)
            i+=1



def main():
    global pub_intermediate_state,pub_joint,pub_drop_state
    # Create publisher
    pub_joint = rospy.Publisher(
        'desired_joint_states', # Topic name
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

