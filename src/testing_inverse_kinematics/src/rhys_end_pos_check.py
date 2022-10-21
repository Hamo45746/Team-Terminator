#!/usr/bin/python3
# Always need this

import rospy

# Import message types
import std_msgs.msg as msg
from std_msgs import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

# inverse kinematics stuff
import numpy as np
import math as m

print('end pos checker running')

L1 = 0.055
L2 = 0.118
L3 = 0.095
L4 = 0.07



joint_state = [1000,1000,1000,1000]

def store_is_moving(i):
    global is_moving
    is_moving = i.data

def store_grab(pose: JointState):
    global grab_state
    grab_state= pose
    

def store_colour_state(pose: JointState):
    global colour_state
    colour_state= pose

def store_drop(pose: JointState):
    global drop_state
    if abs(pose.position[0]) > 0.1:
        drop_state= pose     


def store_colour(i):
    global colour
    colour = i.data


def store_state(i):
    global state
    state = i.data
 
def end_pos_check (des_thetaL, act_thetaL, cfl, cfh):
    
    for i in range(4):
        des_thetaL[i] = des_thetaL[i]*(180/m.pi)
        act_thetaL[i] = act_thetaL[i]*(180/m.pi)
    percent_dif_list = [0,0,0,0]
    
    for i, theta in enumerate(act_thetaL):
        if (-2 <= theta <= 2) and (-2 <= des_thetaL[i] <= 2):
            percent_dif_list[i] = 1
        elif (des_thetaL[i]== 0) and (theta != 0):
            des_thetaL[i] = 1
            percent_dif_list[i] = abs(act_thetaL[i]/des_thetaL[i])
        elif (theta == 0) and (des_thetaL[i] != 0):
            
            act_thetaL[i] = 1
            percent_dif_list[i] = abs(act_thetaL[i]/des_thetaL[i])
            percent_dif_list[i] = 1
        else:
             percent_dif_list[i] = act_thetaL[i]/des_thetaL[i]


    if (cfl <= percent_dif_list[0] <= cfh) and (cfl <= percent_dif_list[1] <= cfh) and (cfl <= percent_dif_list[2] <= cfh) and (cfl <= percent_dif_list[3] <= cfh):
        print("true")
        check = True
        return check     
    else:
        check = False
        print('false')
        return check

    

def verify(joint_state: JointState): # passes in the desired state
    print("verify")

    global grab_state, colour_state,drop_state, state,pub_close,is_moving
   
    if len(joint_state.position) == 4:
        
        #assign variables
        act_thetaL = [joint_state.position[3],joint_state.position[2],joint_state.position[1],joint_state.position[0]]
        #desired theta list [ theta1, theta2, theta3, theta4]
        if state == 0:

            cfl = 0.9 #close factor low
            cfh = 1.1 #close factor high
            des_thetaL = [0,0,0,0] #desired theta list [ theta1, theta2, theta3, theta4]
            correct_pos = end_pos_check(des_thetaL, act_thetaL, cfl, cfh)
            if correct_pos == True:
                rospy.sleep(2)
                pub_state.publish(1)

        elif state == 1:
            cfl = 0.9 #close factor low
            cfh = 1.3 #close factor high
            des_thetaL = [0, 0.72,-1.44,0.112] #desired theta list [ theta1, theta2, theta3, theta4]
            correct_pos = end_pos_check(des_thetaL, act_thetaL, cfl, cfh)
            if correct_pos == True:
                if is_moving == 0:
                    i=0
                    while i<2:
                        pub_state.publish(2)
                        i+=1

        elif state == 2:
            cfl = 0.96 #close factor low
            cfh = 1.2#close factor high
            des_thetaL = [grab_state.position[0], grab_state.position[1],grab_state.position[2],grab_state.position[3]]
            #desired theta list [ theta1, theta2, theta3, theta4]

            correct_pos = end_pos_check(des_thetaL, act_thetaL, cfl, cfh)
            if correct_pos == True:
                pub_state.publish(3)

        elif state == 4:
            
            cfl = 0.9 #close factor low
            cfh = 1.5 #close factor high
            des_thetaL = [colour_state.position[0],colour_state.position[1],colour_state.position[2],colour_state.position[3]] #[0, -1,0.35,-0.5] #[grab_state.position[0], grab_state.position[1],grab_state.position[2],grab_state.position[3]]
            #desired theta list [ theta1, theta2, theta3, theta4]
            
            correct_pos = end_pos_check(des_thetaL, act_thetaL, cfl, cfh)
            if correct_pos == True:
                i=0  
                rospy.sleep(1)
                pub_state.publish(5)

        elif state == 5:
            
            cfl = 0.9 #close factor low
            cfh = 1.1 #close factor high
            des_thetaL = [drop_state.position[0], drop_state.position[1],drop_state.position[2],drop_state.position[3]]
            #desired theta list [ theta1, theta2, theta3, theta4]
            
            correct_pos = end_pos_check(des_thetaL, act_thetaL, cfl, cfh)
            if correct_pos == True:
                pub_state.publish(6)

        


def main():
    global pub_state,pub_close
    #pub_joint
    
    print('here')
    # Create publisher
    pub_state = rospy.Publisher(
        'state', # Topic name
        msg.Int16, # Message type
        queue_size=10 # Topic size (optional)
    )

    pub_close = rospy.Publisher(
        'end_pos_check', # Topic name
        msg.Int16, # Message type
        queue_size=10 # Topic size (optional)
    )
    '''
    pub_joint = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )
    #'''
    # Create subscriber
    sub = rospy.Subscriber(
        'desired_grab_states', # Topic name
        JointState, # Message type
        store_grab # Callback function (required)
    )

    sub = rospy.Subscriber(
        'desired_colour_states', # Topic name
        JointState, # Message type
        store_colour_state # Callback function (required)
    )
    #'''
    sub = rospy.Subscriber(
        'desired_drop_states', # Topic name
        JointState, # Message type
        store_drop # Callback function (required)
    )
    '''
    sub = rospy.Subscriber(
        'desired_wait_states', # Topic name
        JointState, # Message type
        store_wait # Callback function (required)
    )
    #'''
    sub = rospy.Subscriber(
        'state', # Topic name
        msg.Int16, # Message type
        store_state # Callback function (required)
    )

    sub = rospy.Subscriber(
        'joint_states', # Topic name
        JointState, # Message type
        verify # Callback function (required)
    )

    sub = rospy.Subscriber(
        'is_moving', # Topic name
        msg.Int8, # Message type
        store_is_moving # Callback function (required)
    )

    # Initialise node with any node name
    rospy.init_node('checker')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()