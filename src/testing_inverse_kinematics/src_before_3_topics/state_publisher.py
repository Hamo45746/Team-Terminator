# Always need this
import rospy

# Import message types
import std_msgs.msg as msg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# inverse kinematics stuff
import numpy as np
import math as m


'''
states:
1. searching for cube, waiting for coordinates
2. move to grab
3. grab
4. move to intermdediate
5. move to dropoff (horizontal)
6. move to dropoff forward and back
7. drop it
8. move to inermediate at front

Topics that will dicate what the state changes:



#'''




def iterate_state():
    pass ######################################################################
    



    pub = msg.Int16(



    )


    




if __name__ == '__main__':
    state = 1
    global pub
    pub = rospy.Publisher(
        'state', # Topic name
        msg.Int16, # Message type
        queue_size=10 # Topic size (optional)
    )

    '''
    sub_confirm_point_found = rospy.Subscriber(
        'confirm_point_found', # Topic name
        msg.Int16, # Message type
        iterate_state # Callback function (required)
    )
    #'''

    sub_confirm_grab_angles = rospy.Subscriber(
        'confirm_grab_angles', # Topic name
        msg.Int16, # Message type
        iterate_state # Callback function (required)
    )

    sub_confirm_grab_angles = rospy.Subscriber(
        'confirm_grab_angles', # Topic name
        msg.Int16, # Message type
        iterate_state # Callback function (required)
    )
    '''
    sub_confirm_intermediate_position_angles = rospy.Subscriber(
        'confirm_intermediate_position_angles', # Topic name
        msg.Int16, # Message type
        iterate_state # Callback function (required)
    )

    sub_confirm_horizontal_angle = rospy.Subscriber(
        'confirm_horizontal_angle', # Topic name
        msg.Int16, # Message type
        iterate_state # Callback function (required)
    )

    sub_confirm_drop_angles = rospy.Subscriber(
        'confirm_drop_angles', # Topic name
        msg.Int16, # Message type
        iterate_state # Callback function (required)
    )
    #'''



    






    # Initialise node with any node name
    rospy.init_node('gripper')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()







