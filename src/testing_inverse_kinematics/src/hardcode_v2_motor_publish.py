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

def store_position(i):
    global desired_position
    print('received')
    desired_position = i

def store_colour(i):
    global colour
    colour = i.data


def store_state(i):
    global state
    state = i.data

def store_valid_cube(i):
    global is_valid_cube
    is_valid_cube = i.data

def store_is_moving(i):
    global is_moving
    is_moving = i.data

previous_x = 100
previous_y = 0
previous_z = 0
def inverse_kinematics(pose):
    # pose in this case is a  joint states type, not pose type
    global pub_joint,colour,state,pub_state,desired_position,previous_x,previous_y,previous_z,is_moving,is_valid_cube
    
    L1 = 0.055
    L2 = 0.118
    L3 = 0.094
    L4 = 0.1
    robot_origin = [0 , 0 , 0]
    
    if state == 0:
        msg = JointState(
                header=Header(stamp=rospy.Time.now()),
                name=['joint_1','joint_2', 'joint_3', 'joint_4']
            )
        msg.position = [0,0,0,0]
        pub_joint.publish(msg)
    #print('\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n')



    elif (state==-1): # intermediate state
        
        theta1 = 0
        theta2 = -0.65475068
        theta3 = -1.1464
        theta4 = 0.87010

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
            pub_drop_state.publish(msg)
            i += 1


    elif (state==1): # intermediate state
        
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
            pub_wait_state.publish(msg)
            i += 1

    elif state == 7:
        desired_x = desired_position.position.x
        desired_y = desired_position.position.y  #+ 0.025 + desired_position.position.y*0.11  
        desired_z = desired_position.position.z # elevation
        
        deg_0_threshhold = 0.19
        deg_45_thresh = 0.16

        min_x = 0.08
        desired_distance = np.sqrt(desired_x**2 + desired_y**2) # + 0.015 - this was done for calibration on rig ########
        if desired_distance < deg_45_thresh:
            desired_end_angle = -90 * (np.pi/180)
            # for the actual rig: desired_z=-0.01
            # final values for actual rig desired_z=-0.01   ##########
            desired_z = 0
        elif (deg_45_thresh <= desired_distance <= deg_0_threshhold):
            desired_end_angle = -90 * (np.pi/180)
            # for the actual rig: desired_z=-0.02
            # final values for actual rig desired_z=-0.01 ##########
            desired_z = 0
        elif (desired_distance > deg_0_threshhold):
            desired_end_angle = -5 * (np.pi/180)
            # for the actual rig: desired_z=0.02
            # final values for actual rig desired_z=0.02    ##########
            desired_z = 0
        elif (desired_distance > deg_0_threshhold+0.03):
            desired_end_angle = -5 * (np.pi/180)
            # for the actual rig: desired_z=0.02
            # final values for actual rig desired_z=0.03   ##########
            desired_z = 0
        #try:
        theta1 = np.arctan(desired_y/desired_x)
        #print('\n\n\n\ndesired_y, theta1 ',desired_y,theta1,'\n\n\n')

        #desired_distance = np.sqrt(desired_x**2 + desired_y**2)
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
        #if (desired_x != previous_x) and (desired_y != previous_y) and (desired_z != previous_z):
        print('\n\n\n\n\n\n','succesful','\n\n\n\n\n\n')
        if is_valid_cube == 1:
            i=0
            while (i < 30):
                pub_joint.publish(msg)
                pub_grab_state.publish(msg)
                i+=1
        #except:
        #        print("\n\n\nmath was wrong\n\n\n")
        print('comp',previous_x,desired_x)
        print(previous_y,desired_y)
        previous_x = desired_x
        previous_y = desired_y
        previous_z = desired_z



    if (state == 4): # intermediate state
        
        '''
        desired_x = pose.position.x
        desired_y = pose.position.y
        desired_z = pose.position.z # elevation

        desired_end_angle = 60 * (m.pi/180)
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
        '''
        msg = JointState(
                header=Header(stamp=rospy.Time.now()),
                name=['joint_1','joint_2', 'joint_3', 'joint_4']
            )
        '''
        msg.position = [
            theta1,
            theta2,
            theta3,
            theta4
            ]
        #'''
        msg.position = [0.03,-0.711,-0.4244,-1.279]
        i=0
        while (i < 30):
            pub_joint.publish(msg)
            pub_colour_state.publish(msg)
            i+=1


    elif state == 5:
        #colour = 'B'
        if colour == "B":
            theta1 = -0.2
        elif colour == "R":
            theta1 = 0.2
        elif colour == "G":
            theta1 = -1.33
        elif colour == "Y":
            theta1 = 1.33
        else:
            pub_state.publish(-1)

        ''' These were for not flipping over
        theta2 = -0.472
        theta3 = -1.52
        theta4 = 0.578
        '''

        theta2 = 0.48
        theta3 = 1.54
        theta4 = -0.92
        


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
        i=0
        while (i < 30):
            pub_joint.publish(msg)
            pub_drop_state.publish(msg)
            i+=1

    #'''
    

        

def main():
    global pub_colour_state,pub_joint,pub_drop_state,pub_grab_state, pub_wait_state,pub_state
    # Create publisher
    pub_joint = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=50 # Topic size (optional)
    )

    pub_wait_state = rospy.Publisher(
        'desired_wait_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )

    pub_grab_state = rospy.Publisher(
        'desired_grab_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )

    pub_colour_state = rospy.Publisher(
        'desired_colour_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )

    pub_drop_state = rospy.Publisher(
        'desired_drop_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )
  
    pub_state = rospy.Publisher(
        'state', # Topic name
        msg.Int16, # Message type
        queue_size=10 # Topic size (optional)
    )

    sub = rospy.Subscriber(
        'is_moving', # Topic name
        msg.Int8, # Message type
        store_is_moving # Callback function (required)
    )

    # Create subscriber
    sub = rospy.Subscriber(
        'desired_position', # Topic name
        Pose, # Message type
        store_position # Callback function (required)
    )
    #'''

    sub = rospy.Subscriber(
        'joint_states', # Topic name
        JointState, # Message type
        inverse_kinematics # Callback function (required)
    )
    
    sub = rospy.Subscriber(
        'state', # Topic name
        msg.Int16, # Message type
        store_state # Callback function (required)
    )

    sub = rospy.Subscriber(
        'is_valid_cube', # Topic name
        msg.Int8, # Message type
        store_valid_cube # Callback function (required)
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
