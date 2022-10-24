#!/usr/bin/python3

import rospy
import pigpio
# Import message types
import std_msgs.msg as msg
from sensor_msgs.msg import JointState


rpi = pigpio.pi()
rpi.set_mode(18, pigpio.OUTPUT)

def publish(state, iterations):
    i = 0
    while (i < iterations):
        pub.publish(state)
        i+=1

def store_state(i):
    global state
    state = i.data
    

def move_gripper(i): 
    '''
    Function to move the gripper to the open or closed positition depending on \
        the current state then publish the next state
    Inputs:
    - i (JointState): JointState object containing the current joint angles
    Outputs:
    - N/A

    Note that:
    1000 is the closed position
    1410 is the grip box position
    2000 is the open position
    '''
    #1000 is the closed position
    #1500 is the grip box position
    #2000 is the open position
    global state
    grab = 1410
    open = 2000
    
    if state == 3:
        rospy.sleep(0.5)
        rpi.set_servo_pulsewidth(18,grab)
        rospy.sleep(0.7)
        publish(8,40)
    elif (state == 4):
        # If the angles is less than 0.85, then release the gripper and throw \
        # the cube
        if abs(i.position[1]) < 0.85:
            rpi.set_servo_pulsewidth(18,open)
            publish(6,10)
    elif (state == 6):
        rpi.set_servo_pulsewidth(18,open)
        publish(1,10)
    elif state == 1:
        rpi.set_servo_pulsewidth(18,open)

if __name__ == '__main__':
    '''
    Define the publishers and subscribers required for this script
    Publishers:
    - pub_state: Publish the desired state
    Subscribers:
    - state_sub: retrieve the current state
    - joint_sub: retrieve the current joint angles
    '''
    global pub
    # Create publisher
    pub = rospy.Publisher('state', msg.Int16, queue_size=10)

    # Create subscriber
    state_sub = rospy.Subscriber('state', msg.Int16, store_state)
    joint_sub = rospy.Subscriber('joint_states', JointState, move_gripper)

    rospy.init_node('gripper_publisher_throw')
    rospy.spin()
