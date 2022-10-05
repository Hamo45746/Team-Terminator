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


import rospy
import cv2
import numpy as np 
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge, CvBridgeError

'''
GET GRIPPER TO LOOK AT STATE SO DOESNT OPEN WHEN MOVEING TO INTERMEDIATE ETC
'''
print('camera running')

def store_state(i):
    global state
    state = i.data



bridge = CvBridge()
def publish_colour(i): 
    global state,bridge,colour_pub

    # logic if state==4
    if 4==4:
        try:
            img = bridge.imgmsg_to_cv2(i, "bgr8")
            bgr = img[img.shape[0] // 2, img.shape[1] // 2, :]

            rgbcolour = ""

            colour = ColorRGBA()
            colour.r = bgr[2]
            colour.g = bgr[1]
            colour.b = bgr[0]

            # ologic f0to detect colour of the block at the giuven pixel

            if(colour.r > 150 and colour.g > 150 and colour.b < 150):
                rgbcolour =  "Y"
            elif(colour.r > 150 and colour.g < 150 and colour.b < 150):
                rgbcolour = "R"
            elif(colour.r < 150 and colour.g > 150 and colour.b < 150):
                rgbcolour = "G"
            elif(colour.r < 100 and colour.g > 100 and colour.b > 100):
                rgbcolour = "B"
            colour_pub.publish(rgbcolour)
        except:
            pass
    
    



if __name__ == '__main__':
    global colour_pub,sub #viewer
    # Create publisher
    #viewer = CameraViewer('31704051')

    sub = rospy.Subscriber(
        'state', # Topic name
        msg.Int16, # Message type
        store_state # Callback function (required)
    )

    image_sub = rospy.Subscriber(f"/ximea_ros/ximea_31704051/image_raw", Image, publish_colour)
    colour_pub = rospy.Publisher("colour", msg.String, queue_size=10)



    # Create subscriber
    

    # Initialise node with any node name
    rospy.init_node('camera')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()

'''
if __name__ == '__main__':
    main()
#'''










