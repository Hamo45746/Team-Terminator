#!/usr/bin/python3
# Always need this
import rospy
import pigpio
# Import message types
import std_msgs.msg as msg

# inverse kinematics 
import math as m

from fiducial_msgs.msg import FiducialArray

import rospy

import numpy as np 


'''
GET GRIPPER TO LOOK AT STATE SO DOESNT OPEN WHEN MOVEING TO INTERMEDIATE ETC
'''
print('moving checker running')


old_pos_dict = {}

def determine_if_moving(i):
    global vertex
    vertex = i
    '''
    #print(len(vertex.fiducials),"\n\n")
    print(vertex.fiducials[0].fiducial_id)
    print(vertex.fiducials[1].fiducial_id)
    print(vertex.fiducials[2].fiducial_id,"\n\n\n\n")
    #hile not_same_tag_found = :
    #'''
    
    old_corners = [[0,0],[0,0],[0,0],[0,0]]
    if vertex.fiducials != []:
        for tag in vertex.fiducials:
            lower_bound = 0.99
            upper_bound = 1.01

            if tag.fiducial_id in old_pos_dict:
                old_corners = old_pos_dict.get(tag.fiducial_id) #returns [[old_x0,old_y0],[old_x1,old_y1],[old_x2,old_y2],[old_x3,old_y3]]

                #print("old_corners[0][0]: ", old_corners[0])
                #print("old_corners[0][1]: ", old_corners[0][1])
                #print("tag.x0: ", tag.x0)
                #print("tag.y0: ", tag.y0)

                if (old_corners[0][0]*lower_bound <= tag.x0 <= old_corners[0][0]*upper_bound) and (old_corners[0][1]*lower_bound <= tag.y0 <= old_corners[0][1]*upper_bound):

                    ismoving_pub.publish(0)
                else:
                    ismoving_pub.publish(1)
            #if ([0][0] <= corner[0] <= old_pos_dict.get(tag.fiducial_id))



    for i,tag in enumerate(vertex.fiducials):
        old_pos_dict[tag.fiducial_id] = [[tag.x0,tag.y0],[tag.x1,tag.y1],[tag.x2,tag.y2],[tag.x3,tag.y3]]
    #print(old_pos_dict)

if __name__ == '__main__':
    global ismoving_pub
  
    #image_sub = rospy.Subscriber(f"/ximea_ros/ximea_31704051/image_raw", Image, publish_colour)
    vertex_sub = rospy.Subscriber("fiducial_vertices", FiducialArray , determine_if_moving)


    ismoving_pub = rospy.Publisher("is_moving", msg.Int8, queue_size=10)



    # Create subscriber
    

    # Initialise node with any node name
    rospy.init_node('moving_checker')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()

'''
if __name__ == '__main__':
    main()
#'''










