#!/usr/bin/python3
import rospy
import pigpio
# Import message types
import std_msgs.msg as msg
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# inverse kinematics stuff
import numpy as np
import math as m




def publish_intermediate(i):
    # theta1 is determined by the topic determined by the camera
    if i.data == 1:

        theta2 = 0.71
        theta3 = -2.2
        theta4 = 0


        msg = JointState(
            # Set header with current time
            header=Header(stamp=rospy.Time.now()),
            # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
            name=['joint_1','joint_2', 'joint_3', 'joint_4']
        )



        msg.position = [
            0,
            theta2,
            theta3,
            theta4
            ]   # THIS DOESNT INCLUDE HORIZONTAL  


        pub.publish(msg)

# CAN COMBINE THIS with THE EXISTING PUBLISHER TO MAKE MORE SIMPLE





if __name__ == '__main__':

    sub = rospy.Subscriber('gripper_close_confirmation',msg.Int16,publish_intermediate)

    pub = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )
    rospy.init_node('intermediate_publisher')
    rospy.spin()



