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


from fiducial_msgs.msg import FiducialArray

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

def store_is_moving(i):
    global is_moving
    is_moving = i.data

#################################################################################################################################

vertex_dict = {}

bridge = CvBridge()
def publish_colour(i): 
    global state,bridge,colour_pub,vertex,is_moving,pub_state
    vertex_dict = {}
    for i,tag in enumerate(vertex.fiducials):
        vertex_dict[tag.fiducial_id] = [[tag.x0,tag.y0],[tag.x1,tag.y1],[tag.x2,tag.y2],[tag.x3,tag.y3]]

    lowest = 10000
    id_tag = 0
    angle_dict = {}
    for tag in vertex_dict:
        x0 = vertex_dict.get(tag)[0][0]
        x1 = vertex_dict.get(tag)[1][0]
        y0 = vertex_dict.get(tag)[0][1]
        y1 = vertex_dict.get(tag)[1][1]
        x2 = vertex_dict.get(tag)[2][0]
        y2 = vertex_dict.get(tag)[2][1]

        angle_dict[tag] = min(abs(m.atan((y1-y0) / (x1-x0)) * (180/np.pi)),abs((y2-y1) / (x2-x1) * (180/np.pi)))
        if angle_dict.get(tag) < lowest:
            lowest = angle_dict.get(tag)
            id_tag = tag

    # publish the angles in cube frame

    '''
    lowest = 10000
    for tag in vertex_dict:
        if lowest > angle_dict.get(tag):
            lowest = angle_dict.get(tag)
    for tag in vertex
    #'''

    #print(angle_dict.keys()[min(angle_dict.values())])
    #print(list(angle_dict.keys())[list(min(angle_dict.values()))])

    # logic if state==4
    if state==4:
        try:
            img = bridge.imgmsg_to_cv2(i, "bgr8")
            bgr = img[img.shape[0] // 2, img.shape[1] // 2, :]
            #print(img.shape[0] // 2, img.shape[1] // 2)
            rgbcolour = ""

            colour = ColorRGBA()
            r = bgr[2]
            g = bgr[1]
            b = bgr[0]
            # normalise the rgb values
            norm_r = r / max(r,b,g)
            norm_g = g / max(r,b,g)
            norm_b = b / max(r,b,g)
            print(norm_r)
            print(norm_g)
            print(norm_b)

            # ologic f0to detect colour of the block at the giuven pixel
            '''
            if(colour.r > 150 and colour.g > 150 and colour.b < 150):
                rgbcolour =  "Y"
            elif(colour.r > 150 and colour.g < 150 and colour.b < 150):
                rgbcolour = "R"
            elif(colour.r < 150 and colour.g > 150 and colour.b < 150):
                rgbcolour = "G"
            elif(colour.r < 100 and colour.g > 100 and colour.b > 100):
                rgbcolour = "B"
            colour_pub.publish(rgbcolour)
            #'''
            # using normalised values
            if(norm_r >0.8 and norm_g >0.8 and norm_b <0.4):
                rgbcolour = "Y"
            elif (norm_r == 1 and norm_g > 0.2 and norm_b < 0.3):
                rgbcolour =  "R"
            elif(norm_r >0.38 and norm_g ==1 and norm_b < 0.4):
                rgbcolour = "G"
            elif(norm_r <0.45 and norm_g >0.6 and norm_b == 1):
                rgbcolour = "B"
            
            colour_pub.publish(rgbcolour)
        except:
            pass
    # elif state==1:
    #     # check is moving
    #     if is_moving == 1:
    #         # changwe stte
    #         pub_state.publish(2)

def store_vertex(i):
    global vertex
    vertex = i



if __name__ == '__main__':
    global colour_pub,sub,pub_state #viewer
    # Create publisher
    #viewer = CameraViewer('31704051')

    sub = rospy.Subscriber(
        'state', # Topic name
        msg.Int16, # Message type
        store_state # Callback function (required)
    )
    sub = rospy.Subscriber(
        'is_moving', # Topic name
        msg.Int8, # Message type
        store_is_moving # Callback function (required)
    )

    image_sub = rospy.Subscriber(f"/ximea_ros/ximea_31704051/image_raw", Image, publish_colour)
    vertex_sub = rospy.Subscriber("fiducial_vertices", FiducialArray , store_vertex)

    colour_pub = rospy.Publisher("colour", msg.String, queue_size=10)
    pub_state = rospy.Publisher("state", msg.Int16, queue_size=10)


    # Create subscriber
    

    # Initialise node with any node name
    rospy.init_node('camera_node')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()

'''
if __name__ == '__main__':
    main()
#'''










