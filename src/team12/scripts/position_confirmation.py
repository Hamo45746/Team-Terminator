#!/usr/bin/python3

import rospy
import std_msgs.msg as msg
from std_msgs import *
from sensor_msgs.msg import JointState
import numpy as np

joint_state = [1000,1000,1000,1000]

def store_is_valid(i):
    '''
    Stores whether cube is in valid position to be grabbed. 
    1 for valid position, 0 for invalid position.
    inputs: i (int)
    outputs: N/A
    '''
    global is_valid_cube
    is_valid_cube = i.data

def store_is_moving(i):
    '''
    Stores whether cube is moving or not.
    1 for moving, 0 for stationary.
    inputs: i (int)
    outputs: N/A
    '''
    global is_moving
    is_moving = i.data

def store_grab(pose: JointState):
    '''
    Stores desired joint states at which to close gripper.
    inputs: pose (JointState)
    outputs: N/A
    '''
    global grab_state
    grab_state= pose
    
def store_inter_state(pose: JointState):
    '''
    Stores desired joint states to reach between grabbing the cube and
    holding it up to the camera.
    inputs: pose (JointState)
    output: N/A
    '''
    global inter_state
    inter_state = pose

def store_colour_state(pose: JointState):
    '''
    Stores desired joint states to hold cube up to camera.
    inputs: pose (JointState)
    outputs: N/A
    '''
    global colour_state
    colour_state= pose

def store_drop(pose: JointState):
    '''
    Stores desired joint states to drop cube at, 
    depending on the detected colour.
    inputs: pose (JointState)
    output: N/A
    '''
    global drop_state
    if abs(pose.position[0]) > 0.1:
        drop_state= pose     

def store_colour(i):
    '''
    Stores the colour of the targeted cube.
    inputs: i (String, one of 'Y', 'G', 'R', 'B')
    outputs: N/A
    '''
    global colour
    colour = i.data

def store_state(i):
    '''
    Stores the current state of the robot.
    inputs: i (int)
    outputs: N/A
    '''
    global state
    state = i.data
 
def end_pos_check (des_thetaL, act_thetaL, cfl, cfh):
    '''
    Checks whether the actual joint angles are within a range of the 
    desired joint angles.
    inputs: 
        - des_thetaL (list) desired joint angles
        - act_thetaL (list) actual joint angles
        - cfl (float) low percentage threshold of joint angles
        - cfh (float) high percentage threshold of joint angles
    outputs: (boolean) True if actual angles are within threshold,
                False if actual angles are outside threshold
    '''
    des_thetaL = np.rad2deg(des_thetaL)    
    act_thetaL = np.rad2deg(act_thetaL)
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

    if (cfl <= percent_dif_list[0] <= cfh) and \
            (cfl <= percent_dif_list[1] <= cfh) and \
            (cfl <= percent_dif_list[2] <= cfh) and \
            (cfl <= percent_dif_list[3] <= cfh):
        return True     
    else:
        return False

def verify(joint_state: JointState): # passes in the desired state
    '''
    Checks if the actual joint states match the desired joint states,
    depending on what position is wanted in each state. Publishes
    the next state if the end position of current state is reached.
    inputs: joint_state (JointState)
    outputs: N/A
    '''
    global grab_state, colour_state,drop_state, state,pub_close,is_valid_cube,\
        is_moving,inter_state
   
    if len(joint_state.position) == 4:
        act_thetaL = [joint_state.position[3],joint_state.position[2],\
            joint_state.position[1],joint_state.position[0]]

        if state == -1:
            cfl = 0.96 #close factor low
            cfh = 1.5 #close factor high
            des_thetaL = [grab_state.position[0], grab_state.position[1],\
                grab_state.position[2],grab_state.position[3]]
            correct_pos = end_pos_check(des_thetaL, act_thetaL, cfl, cfh)
            if correct_pos == True:
                pub_state.publish(6)
        elif state == 0:
            cfl = 0.9 #close factor low
            cfh = 1.1 #close factor high
            des_thetaL = [0,0,0,0] #desired theta list
            correct_pos = end_pos_check(des_thetaL, act_thetaL, cfl, cfh)
            if correct_pos == True:
                rospy.sleep(2)
                pub_state.publish(1)
        elif state == 1:
            cfl = 0.8 #close factor low
            cfh = 1.3 #close factor high
            des_thetaL = [0, 0.72,-1.44,0.112] #desired theta list
            correct_pos = end_pos_check(des_thetaL, act_thetaL, cfl, cfh)
            if correct_pos == True:
                if is_moving == 0:
                    i=0
                    while i<2:
                        pub_state.publish(2)
                        i+=1
        elif state == 7:
            cfl = 0.9 #close factor low
            cfh = 1.5#close factor high
            des_thetaL = [grab_state.position[0], grab_state.position[1],\
                grab_state.position[2],grab_state.position[3]]
            correct_pos = end_pos_check(des_thetaL, act_thetaL, cfl, cfh)
            if correct_pos == True:
                pub_state.publish(3)
        elif state == 4:
            cfl = 0.9 #close factor low
            cfh = 1.5 #close factor high
            des_thetaL = [0.03,-0.711,-0.4244,-1.279] 
            
            correct_pos = end_pos_check(des_thetaL, act_thetaL, cfl, cfh)
            if correct_pos == True:
                i=0  
                rospy.sleep(2)
                pub_state.publish(5)
        elif state == 5:
            cfl = 0.95 #close factor low
            cfh = 1.1 #close factor high
            des_thetaL = [drop_state.position[0], drop_state.position[1],\
                drop_state.position[2],drop_state.position[3]]
            
            correct_pos = end_pos_check(des_thetaL, act_thetaL, cfl, cfh)
            if correct_pos == True:
                pub_state.publish(6)
        elif state == 8:
            cfl = 0.6 #close factor low
            cfh = 1.5 #close factor high
            des_thetaL = [inter_state.position[0], inter_state.position[1],\
                inter_state.position[2],inter_state.position[3]]

            correct_pos = end_pos_check(des_thetaL, act_thetaL, cfl, cfh)
            if correct_pos == True:
                pub_state.publish(4)

if __name__ == '__main__':
    global pub_state,pub_close,state
    state = 0
    is_valid_cube = 1
    
    # Create publisher
    pub_state = rospy.Publisher('state', msg.Int16, queue_size=10)
    pub_close = rospy.Publisher('end_pos_check', msg.Int16, queue_size=10)

    # Create subscriber
    sub = rospy.Subscriber('desired_grab_states', JointState, store_grab)
    sub = rospy.Subscriber('desired_colour_states', JointState,\
        store_colour_state)    
    sub = rospy.Subscriber('desired_drop_states', JointState, store_drop)
    sub = rospy.Subscriber('inter_state', JointState, store_inter_state)
    sub = rospy.Subscriber('state', msg.Int16, store_state)
    sub = rospy.Subscriber('joint_states', JointState, verify)
    sub = rospy.Subscriber('is_valid_cube', msg.Int8, store_is_valid)
    sub = rospy.Subscriber('is_moving', msg.Int8, store_is_moving)

    rospy.init_node('position_confirmation')
    rospy.spin()