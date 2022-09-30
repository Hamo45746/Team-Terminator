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





rpi = pigpio.pi()
rpi.set_mode(18, pigpio.OUTPUT)

print('gripper running')

def move_gripper(i): 
    global been1
    grab = 1450
    open = 2000
    
    if i.data == (3):
        print('gripper')
        # close gripper (grab block)
        rpi.set_servo_pulsewidth(18,grab)
        new_state = 4
        pub.publish(new_state)

    elif (i.data == 6):
        rpi.set_servo_pulsewidth(18,open)
        new_state = 1
        pub.publish(new_state)
    #1000 is the closed position
    #1500 is the grip box position
    #2000 is the open position



if __name__ == '__main__':
    global pub
    # Create publisher


    # Create subscriber
    sub = rospy.Subscriber(
        'state', # Topic name
        msg.Int16, # Message type
        move_gripper # Callback function (required)
    )
    pub = rospy.Publisher(
        'state', # Topic name
        msg.Int16, # Message type
        queue_size=10 # Topic size (optional)
    )

    # Initialise node with any node name
    rospy.init_node('gripper')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()

'''
if __name__ == '__main__':
    main()
#'''










