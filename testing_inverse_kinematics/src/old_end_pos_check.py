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

print('checker running')

L1 = 0.055
L2 = 0.118
L3 = 0.095
L4 = 0.07



joint_state = [1000,1000,1000,1000]


def store_grab(pose: JointState):
    global grab_state
    grab_state= pose
    

def store_intermediate(pose: JointState):
    global int_state
    int_state= pose

def store_drop(pose: JointState):
    global drop_state
    drop_state= pose    


state = 2

been_1q_g = 0
been_1q_i = 0
been_1q_d = 0
been_0q = 0

def store_colour(i):
    global colour
    colour = i.data


def store_state(i):
    global state
    state = i.data


def verify(joint_state: JointState): # passes in the desired state
    print("verify")
    global grab_state, int_state,drop_state, state,been_1q_g,been_1q_i,been_1q_d
    if state==2:
        current_theta4 = joint_state.position[0]
        current_theta3 = joint_state.position[1]
        current_theta2 = joint_state.position[2]
        current_theta1 = joint_state.position[3]

        desired_theta1 = grab_state.position[0]
        desired_theta2 = grab_state.position[1]
        desired_theta3 = grab_state.position[2]
        desired_theta4 = grab_state.position[3]

        close_factor_low = 1
        close_factor_high = 1.04

        if desired_theta1 != float(0):
            theta1_close_factor = np.abs(current_theta1 / desired_theta1)
            theta2_close_factor = np.abs(current_theta2 / desired_theta2)
            theta3close_factor = np.abs(current_theta3 / desired_theta3)
            theta4_close_factor = np.abs(current_theta4 / desired_theta4)

            average = (theta1_close_factor + theta2_close_factor + theta3close_factor + theta4_close_factor) / (4)
            print(average)
        else:
            theta2_close_factor = np.abs(current_theta2 / desired_theta2)
            theta3close_factor = np.abs(current_theta3 / desired_theta3)
            theta4_close_factor = np.abs(current_theta4 / desired_theta4)
            average = (theta2_close_factor + theta3close_factor + theta4_close_factor) / (3)
            print('hi')
        
        if (average > close_factor_low) and (average < close_factor_high) and (been_1q_g==0):
            print('close')
            if state == 2:
                pub_state.publish(3)
                state = 3
            elif state==4:
                pub_state.publish(5)
                state = 5
            elif state==5:
                pub_state.publish(6)
                state = 6
            been_1q_g=1
            '''
            if state == 4:
                print('here')
                theta2 = 0.71
                theta3 = -2.2
                theta4 = 0
                theta1 = 0
            elif state == 5:
                colour = "blue"
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
            # define the msg to be published
            if state in [4,5]:
                msg = JointState(
                        # Set header with current time
                        header=Header(stamp=rospy.Time.now()),
                        name=['joint_1','joint_2', 'joint_3', 'joint_4']
                    )

                msg.position = [
                    theta1,
                    theta2,
                    theta3,
                    theta4
                    ]   # THIS DOESNT INCLUDE HORIZONTAL  

                pub_joint.publish(msg)
                while current_theta1 < np.abs(desired_theta1*0.95):
                    print('hi')
                    j=1
                #'''
    elif state == 4:
        current_theta4 = joint_state.position[0]
        current_theta3 = joint_state.position[1]
        current_theta2 = joint_state.position[2]
        current_theta1 = joint_state.position[3]

        desired_theta1 = int_state.position[0]
        desired_theta2 = int_state.position[1]
        desired_theta3 = int_state.position[2]
        desired_theta4 = int_state.position[3]


        close_factor_low = 1*126
        close_factor_high = 1.04*126

        if desired_theta1 != float(0):
            theta1_close_factor = np.abs(current_theta1 / desired_theta1)
            theta2_close_factor = np.abs(current_theta2 / desired_theta2)
            theta3close_factor = np.abs(current_theta3 / desired_theta3)
            theta4_close_factor = np.abs(current_theta4 / desired_theta4)

            average = (theta1_close_factor + theta2_close_factor + theta3close_factor + theta4_close_factor) / (4)
            print(average)
            
        else:
            theta2_close_factor = np.abs(current_theta2 / desired_theta2)
            theta3close_factor = np.abs(current_theta3 / desired_theta3)
            theta4_close_factor = np.abs(current_theta4 / desired_theta4)
            average = (theta2_close_factor + theta3close_factor + theta4_close_factor) / (3)
        
        if (average > close_factor_low) and (average < close_factor_high) and (been_1q_i==0):
            print('close')
            if state == 2:
                pub_state.publish(3)
                state = 3
            elif state==4:
                pub_state.publish(5)
                state = 5
            elif state==5:
                pub_state.publish(6)
                state = 6
            been_1q_i=1

    if state==5:
        current_theta4 = joint_state.position[0]
        current_theta3 = joint_state.position[1]
        current_theta2 = joint_state.position[2]
        current_theta1 = joint_state.position[3]

        desired_theta1 = drop_state.position[0]
        desired_theta2 = drop_state.position[1]
        desired_theta3 = drop_state.position[2]
        desired_theta4 = drop_state.position[3]


        close_factor_low = 1
        close_factor_high = 1.04

        if desired_theta1 != float(0):
            theta1_close_factor = np.abs(current_theta1 / desired_theta1)
            theta2_close_factor = np.abs(current_theta2 / desired_theta2)
            theta3close_factor = np.abs(current_theta3 / desired_theta3)
            theta4_close_factor = np.abs(current_theta4 / desired_theta4)

            average = (theta1_close_factor + theta2_close_factor + theta3close_factor + theta4_close_factor) / (4)
            print(average)
        else:
            theta2_close_factor = np.abs(current_theta2 / desired_theta2)
            theta3close_factor = np.abs(current_theta3 / desired_theta3)
            theta4_close_factor = np.abs(current_theta4 / desired_theta4)
            average = (theta2_close_factor + theta3close_factor + theta4_close_factor) / (3)
        
        if (average > close_factor_low) and (average < close_factor_high) and (been_1q_d==0):
            print('close')
            if state == 2:
                pub_state.publish(3)
                state = 3
            elif state==4:
                pub_state.publish(5)
                state = 5
            elif state==5:
                pub_state.publish(6)
                state = 6
            been_1q_d=1

'''
def verify_drop(joint_state: JointState): # passes in the desired state
    print("verify drop")
    global drop_state, state  
    current_theta4 = joint_state[0]
    current_theta3 = joint_state[1]
    current_theta2 = joint_state[2]
    current_theta1 = joint_state[3]

    desired_theta1 = drop_state.position[0]
    desired_theta2 = drop_state.position[1]
    desired_theta3 = drop_state.position[2]
    desired_theta4 = drop_state.position[3]


    close_factor_low = 1
    close_factor_high = 1.04

    if desired_theta1 != float(0):
        theta1_close_factor = np.abs(current_theta1 / desired_theta1)
        theta2_close_factor = np.abs(current_theta2 / desired_theta2)
        theta3close_factor = np.abs(current_theta3 / desired_theta3)
        theta4_close_factor = np.abs(current_theta4 / desired_theta4)

        average = (theta1_close_factor + theta2_close_factor + theta3close_factor + theta4_close_factor) / (4)
        print(average)
    else:
        theta2_close_factor = np.abs(current_theta2 / desired_theta2)
        theta3close_factor = np.abs(current_theta3 / desired_theta3)
        theta4_close_factor = np.abs(current_theta4 / desired_theta4)
        average = (theta2_close_factor + theta3close_factor + theta4_close_factor) / (3)
    
    if (average > close_factor_low) and (average < close_factor_high) and (been_1q_d==0):
        print('close')
        if state == 2:
            pub_state.publish(3)
            state = 3
        elif state==4:
            pub_state.publish(5)
            state = 5
        elif state==5:
            pub_state.publish(6)
            state = 6
        been_1q_d=1






def verify_intermediate(joint_state: JointState): # passes in the desired state
    print("verify inter")

    global int_state, state  
    current_theta4 = joint_state[0]
    current_theta3 = joint_state[1]
    current_theta2 = joint_state[2]
    current_theta1 = joint_state[3]

    desired_theta1 = int_state.position[0]
    desired_theta2 = int_state.position[1]
    desired_theta3 = int_state.position[2]
    desired_theta4 = int_state.position[3]


    close_factor_low = 1
    close_factor_high = 1.04

    if desired_theta1 != float(0):
        theta1_close_factor = np.abs(current_theta1 / desired_theta1)
        theta2_close_factor = np.abs(current_theta2 / desired_theta2)
        theta3close_factor = np.abs(current_theta3 / desired_theta3)
        theta4_close_factor = np.abs(current_theta4 / desired_theta4)

        average = (theta1_close_factor + theta2_close_factor + theta3close_factor + theta4_close_factor) / (4)
        print(average)
    else:
        theta2_close_factor = np.abs(current_theta2 / desired_theta2)
        theta3close_factor = np.abs(current_theta3 / desired_theta3)
        theta4_close_factor = np.abs(current_theta4 / desired_theta4)
        average = (theta2_close_factor + theta3close_factor + theta4_close_factor) / (3)
    
    if (average > close_factor_low) and (average < close_factor_high) and (been_1q_i==0):
        print('close')
        if state == 2:
            pub_state.publish(3)
            state = 3
        elif state==4:
            pub_state.publish(5)
            state = 5
        elif state==5:
            pub_state.publish(6)
            state = 6
        been_1q_i=1



def verify_drop(joint_state: JointState): # passes in the desired state
    print("verify drop")
    global drop_state, state  
    current_theta4 = joint_state[0]
    current_theta3 = joint_state[1]
    current_theta2 = joint_state[2]
    current_theta1 = joint_state[3]

    desired_theta1 = drop_state.position[0]
    desired_theta2 = drop_state.position[1]
    desired_theta3 = drop_state.position[2]
    desired_theta4 = drop_state.position[3]


    close_factor_low = 1
    close_factor_high = 1.04

    if desired_theta1 != float(0):
        theta1_close_factor = np.abs(current_theta1 / desired_theta1)
        theta2_close_factor = np.abs(current_theta2 / desired_theta2)
        theta3close_factor = np.abs(current_theta3 / desired_theta3)
        theta4_close_factor = np.abs(current_theta4 / desired_theta4)

        average = (theta1_close_factor + theta2_close_factor + theta3close_factor + theta4_close_factor) / (4)
        print(average)
    else:
        theta2_close_factor = np.abs(current_theta2 / desired_theta2)
        theta3close_factor = np.abs(current_theta3 / desired_theta3)
        theta4_close_factor = np.abs(current_theta4 / desired_theta4)
        average = (theta2_close_factor + theta3close_factor + theta4_close_factor) / (3)
    
    if (average > close_factor_low) and (average < close_factor_high) and (been_1q_d==0):
        print('close')
        if state == 2:
            pub_state.publish(3)
            state = 3
        elif state==4:
            pub_state.publish(5)
            state = 5
        elif state==5:
            pub_state.publish(6)
            state = 6
        been_1q_d=1

#'''

def main():
    global pub_state
    #pub_joint
    
    # Create publisher
    pub_state = rospy.Publisher(
        'state', # Topic name
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
        'desired_intermediate_states', # Topic name
        JointState, # Message type
        store_intermediate # Callback function (required)
    )

    sub = rospy.Subscriber(
        'desired_drop_states', # Topic name
        JointState, # Message type
        store_drop # Callback function (required)
    )

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



    # Initialise node with any node name
    rospy.init_node('checker')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()














