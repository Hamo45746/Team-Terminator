#!/usr/bin/python3
# Always need this
import rospy

# Import message types
import std_msgs.msg as msg
from fiducial_msgs.msg import FiducialArray
import rospy

old_pos_dict = {}

def determine_if_moving(i):
    '''
    Callback function for vertex_sub that subscribes to fiducial_vertices. 
    Determines if the position of aruco tag vertices has changed by greather than
    1%. Publishes 0 if the tag is not moving and 1 if it is moving.
    inputs: i (FiducialArray, 2D array of floats)
    outputs: 
    '''
    global vertex
    vertex = i

    old_corners = [[0,0],[0,0],[0,0],[0,0]]
    if vertex.fiducials != []:
        for tag in vertex.fiducials:
            lower_bound = 0.9925
            upper_bound = 1.0075

            if tag.fiducial_id in old_pos_dict:
                #returns [[old_x0,old_y0],[old_x1,old_y1],
                # [old_x2,old_y2],[old_x3,old_y3]]
                old_corners = old_pos_dict.get(tag.fiducial_id) 
                if (old_corners[0][0]*lower_bound <= tag.x0 \
                        <= old_corners[0][0]*upper_bound) \
                        and (old_corners[0][1]*lower_bound <= tag.y0 \
                        <= old_corners[0][1]*upper_bound):
                    ismoving_pub.publish(0)
                else:
                    ismoving_pub.publish(1)

    for i,tag in enumerate(vertex.fiducials):
        old_pos_dict[tag.fiducial_id] = [[tag.x0,tag.y0],[tag.x1,tag.y1],
            [tag.x2,tag.y2],[tag.x3,tag.y3]]

if __name__ == '__main__':
    global ismoving_pub
  
    vertex_sub = rospy.Subscriber("fiducial_vertices", FiducialArray , \
        determine_if_moving)
    ismoving_pub = rospy.Publisher("is_moving", msg.Int8, queue_size=10)

    rospy.init_node('movement_checker')
    rospy.spin()
