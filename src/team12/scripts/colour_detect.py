#!/usr/bin/python3
# Always need this
import rospy
import std_msgs.msg as msg
import rospy
import cv2

from fiducial_msgs.msg import FiducialArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# from team12.scripts.joint_publisher import colour_checker 

def store_state(i):
    '''
    Stores the current state
    inputs: i (Int8), the current state
    outputs: N/A
    '''
    global state
    state = i.data

'''
def store_vertex(i):
    global vertex
    vertex = i
'''

def store_is_moving(i):
    '''
    Stores integers published to the "is_moving" state in order to determine \
        if the cube is moving or not
    inputs: 
    - i (String): The String object containing the colour string
    outputs: N/A
    '''
    global is_moving
    is_moving = i.data

def normalize_rgb(bgrlist):
    '''
    Normalises a given list of rgb values. 
    Normalises involves dividing each value by the maximum value in the list
    Inputs:
    - bgrlist (list): List of the detected rgb values
    Outputs:
    - (list): list of the normalised rgb values
    '''
    r = bgrlist[2]
    g = bgrlist[1]
    b = bgrlist[0]

    # Calculate the normalised values
    norm_r = r / max(r,b,g)
    norm_g = g / max(r,b,g)
    norm_b = b / max(r,b,g)

    return [norm_b,norm_g,norm_r]

def check_colour(normbgrlist):
    '''
    Takes in a list of r,g,b values and returns the detected colour depending \
        on the normalised list. 
    Inputs: 
    - normbgrlist (list): List of normalised rgb colour values
    Outputs: 
    - (String) A letter corresponding to the detected colour's first letter
    '''
    # Assign rgb values
    norm_r = normbgrlist[2]
    norm_g = normbgrlist[1]
    norm_b = normbgrlist[0]

    # Logic for detecting colours:
    if(norm_r >0.8 and norm_g >0.8 and norm_b <0.4):
        return "Y"
    elif (norm_r == 1 and norm_g > 0.2 and norm_b < 0.3):
        return "R"
    elif(norm_r >0.38 and norm_g ==1 and norm_b < 0.4):
        return "G"
    elif(norm_r <0.45 and norm_g >0.6 and norm_b == 1):
        return "B"

bridge = CvBridge()

def publish_colour(i): 
    global state,bridge,colour_pub,is_moving
    '''
    Interprets an image and publishes the detected colour value at the middle \
        pixel to the topic "colour"
    Inputs:
    - i (Image): Image object describing the current detected camera frame
    Outputs:
    - N/A
    '''
    if state==4:
        try:
            rgbcolour = ""
            # Generate the pixel rgb values at the centre of the frame
            img = bridge.imgmsg_to_cv2(i, "bgr8")
            bgr = img[img.shape[0] // 2, img.shape[1] // 2, :]

            # Determine the colour
            rgbcolour = check_colour(normalize_rgb(bgr))
            colour_pub.publish(rgbcolour)
        except:
            pass

if __name__ == '__main__':
    global colour_pub,sub,pub_state #viewer
    '''
    Define the publishers and subscribers required for this script
    Publishers:
    - colour_pub: publish colour string to the "colour" topic
    Subscribers:
    - state_sub: retrieve the current state
    - is_moving_sub: subscribe to topic to determine if blocks are mooving
    - image_sub: retrieve the Image objct describing the current frame
    '''

    state_sub = rospy.Subscriber('state', msg.Int16, store_state)
    is_moving_sub = rospy.Subscriber('is_moving', msg.Int8, store_is_moving)
    image_sub = rospy.Subscriber(f"/ximea_ros/ximea_31704051/image_raw", Image, publish_colour)

    colour_pub = rospy.Publisher("colour", msg.String, queue_size=10)
    
    # Initialise colour_detect node
    rospy.init_node('colour_detect')
    rospy.spin()
