#!/usr/bin/python3

import rospy
import pigpio
# Import message types
import std_msgs.msg as msg

# Initialise the gripper servo controller
rpi = pigpio.pi()
rpi.set_mode(18, pigpio.OUTPUT)

def publish(state, iterations):
    '''
    Publish the new state to the "state" topic
    Inputs:
    - state (Int): the new desired state
    - iterations (Int): the number of times to publish the new state
    '''
    i = 0
    while (i < iterations):
        pub_state.publish(state)
        count+=1

def move_gripper(i): 
    '''
    Function to move the gripper to the open or closed positition depending on \
        the current state then publish the next state
    Inputs:
    - i (Int16): Int16  object containing the current state 
    Outputs:
    - N/A

    Note that:
    1000 is the closed position
    1410 is the grip box position
    2000 is the open position
    '''

    # Define grab and close values
    grab = 1410
    open = 2000
    
    # Open or shut the gripper based on state
    # State 3 => close gripper
    # State 6 or 1 => open gripper
    if i.data == 3: 
        rospy.sleep(0.5)
        rpi.set_servo_pulsewidth(18,grab)
        rospy.sleep(0.7)
        publish(8,40)
    elif (i.data == 6):
        rospy.sleep(0.15)
        rpi.set_servo_pulsewidth(18,open)
        publish(1,70)
    elif i.data == 1:
        rospy.sleep(0.15)
        rpi.set_servo_pulsewidth(18,open)

if __name__ == '__main__':
    global pub_state
    '''
    Define the publishers and subscribers required for this script
    Publishers:
    - pub_state: Publish the desired state
    Subscribers:
    - state_sub: retrieve the current state
    '''
    pub_state = rospy.Publisher('state', msg.Int16, queue_size=10)

    state_sub = rospy.Subscriber('state', msg.Int16, move_gripper)

    # Initialise the gripper node
    rospy.init_node('gripper')
    rospy.spin()
