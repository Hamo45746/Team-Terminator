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
    global state
    state = i.data

def store_is_moving(i):
    global is_moving
    is_moving = i.data

def normalize_rgb(bgrlist):
    r = bgrlist[2]
    g = bgrlist[1]
    b = bgrlist[0]

    norm_r = r / max(r,b,g)
    norm_g = g / max(r,b,g)
    norm_b = b / max(r,b,g)

    return [norm_b,norm_g,norm_r]

def check_colour(normbgrlist):
    
    norm_r = normbgrlist[2]
    norm_g = normbgrlist[1]
    norm_b = normbgrlist[0]

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
    global state,bridge,colour_pub,vertex,is_moving,pub_state

    if state==4:
        try:
            rgbcolour = ""
            img = bridge.imgmsg_to_cv2(i, "bgr8")
            bgr = img[img.shape[0] // 2, img.shape[1] // 2, :]

            rgbcolour = check_colour(normalize_rgb(bgr))
            colour_pub.publish(rgbcolour)
        except:
            pass

def store_vertex(i):
    global vertex
    vertex = i

if __name__ == '__main__':
    global colour_pub,sub,pub_state #viewer

    sub = rospy.Subscriber('state', msg.Int16, store_state)
    sub = rospy.Subscriber('is_moving', msg.Int8, store_is_moving)
    image_sub = rospy.Subscriber(f"/ximea_ros/ximea_31704051/image_raw", Image, publish_colour)
    vertex_sub = rospy.Subscriber("fiducial_vertices", FiducialArray , store_vertex)

    colour_pub = rospy.Publisher("colour", msg.String, queue_size=10)
    pub_state = rospy.Publisher("state", msg.Int16, queue_size=10)

    # Initialise node with any node name
    rospy.init_node('colour_detect')
    rospy.spin()
