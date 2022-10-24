#!/usr/bin/python3

from telnetlib import X3PAD
import rospy
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Transform
import numpy as np
import std_msgs.msg as msg
from geometry_msgs.msg import Pose
from fiducial_msgs.msg import FiducialArray

def store_is_moving(i):
    '''
    Stores whether an aruco tag is moving or not
    inputs: i () 1 or 0, moving or not moving
    outputs: N/A
    '''
    global is_moving
    is_moving = i.data

def store_state(i):
    '''
    Stores the current state of the robot
    inputs: i (int32) integer number of the state
    outputs: N/A
    '''
    global state
    state = i.data

def store_vertex(i):
    '''
    Stores a vertex posiiton of an aruco tag
    inputs: i () 
    outputs: N/A
    '''
    global vertex
    vertex = i

previous_pose = Pose()

def create_vertex_list():
    '''
    Creates a list of tuples of the x and y coordinate of the corners of an aruco tag
    input: N/A
    output: vertex_list (2D array of floats)
    '''
    vertex_list = [[0,0],[0,0],[0,0],[0,0]]
    vertex_list[0][0] = vertex.fiducials[0].x0 #x0 position
    vertex_list[0][1] = vertex.fiducials[0].y0 #y0 position
    vertex_list[1][0] = vertex.fiducials[0].x1 #x1 position
    vertex_list[1][1] = vertex.fiducials[0].y1 #y1 position
    vertex_list[2][0] = vertex.fiducials[0].x2 #x2 position
    vertex_list[2][1] = vertex.fiducials[0].y2 #y2 position
    vertex_list[3][0] = vertex.fiducials[0].x3 #x3 position
    vertex_list[3][1] = vertex.fiducials[0].y3 #y3 position
    return vertex_list

def get_closest_vertex(vertex_list):
    '''
    Finds the aruco tag vertex closest to the robot
    inputs: vertex_list (2D array of floats)
    outputs: vertex_closest (2D array of floats)
    '''
    vertex_closest = [0,0]
    vertex_closest[1] = min(vertex_list[0][1], vertex_list[1][1], \
            vertex_list[2][1], vertex_list[3][1])
    if(vertex_closest[1] == vertex_list[0][1]):
        vertex_closest[0] = vertex_list[0][0]
    elif(vertex_closest[1] == vertex_list[1][1]):
        vertex_closest[0] = vertex_list[1][0]
    elif(vertex_closest[1] == vertex_list[2][1]):
        vertex_closest[0] = vertex_list[2][0]
    elif(vertex_closest[1] == vertex_list[3][1]):
        vertex_closest[0] = vertex_list[3][0]
    return vertex_closest

def get_right_vertex(vertex_list):
    '''
    Finds the vertex of an aruco tag furthest to the right
    input: vertex_list (2D array of floats)
    output: vertex_right (2D array of floats)
    '''
    vertex_right = [0,0]
    vertex_right[0] = min(vertex_list[0][0],vertex_list[1][0],\
            vertex_list[2][0],vertex_list[3][0])
    if(vertex_right[0] == vertex_list[0][0]):
        vertex_right[1] = vertex_list[0][1]
    elif(vertex_right[0] == vertex_list[1][0]):
        vertex_right[1] = vertex_list[1][1]
    elif(vertex_right[0] == vertex_list[2][0]):
        vertex_right[1] = vertex_list[2][1]
    elif(vertex_right[0] == vertex_list[3][0]):
        vertex_right[1] = vertex_list[3][1]
    return vertex_right     


def T_C2R(p):
    '''
    Callback function for the subscriber to fiducial_transforms.
    Transforms the fiducial transform in camera frame into the robot frame.
    Performs other operations based on state of the robot. In state 2 will publish desired x,y
    coordinates of end effector in robot frame to reach the optimal aruco tag. 
    In state 7 publishes the current robot pose. In state 4 publishes desired pose for
    holding cube up to camera. 
    inputs: p (FiducialTransformArray)
    outputs: N/A
    '''
    global camera_to_robot_base 
    global pub
    global sub_transform
    global msg,state,vertex,pub_ignore,camera_to_robot,previous_pose,\
        state_pub,pub_valid_cube,current_pose
    
    transform_array = []
    
    if (state==0):
        pass
    elif (p.transforms != []) and (state==2):
        lowest_theta_set = 10000
        lowest_theta = lowest_theta_set
        for T_camtag in p.transforms:
            T_id = T_camtag.fiducial_id
            T_trans = T_camtag.transform.translation
            if (T_id > 9):
                cam_to_robot_x = 0
                cam_to_robot_y = -0.203
                cam_to_robot_z = 0.376

                camera_to_robot =  np.array([[0, 1, 0, cam_to_robot_x],
                                             [1, 0, 0, cam_to_robot_y],
                                             [0, 0, -1, cam_to_robot_z],
                                             [0, 0, 0, 1]])
                camera_to_tag_1 = np.array([[0, 1, 0, T_trans.x],
                                            [1, 0, 0, T_trans.y],
                                            [0, 0, -1, T_trans.z],
                                            [0, 0, 0, 1]])
                
                robot_to_tag_1 = np.matmul(np.linalg.inv(camera_to_robot),\
                    camera_to_tag_1)

                desired_y = robot_to_tag_1[1][3]
                desired_x = robot_to_tag_1[0][3]
                theta1 = np.arctan(desired_y/desired_x)

                vertex_list = create_vertex_list()
                vertex_closest = get_closest_vertex(vertex_list)
                vertex_right = get_right_vertex(vertex_list)
           
                block_theta = abs(np.arctan((vertex_right[1]-\
                    vertex_closest[1]) / (vertex_right[0]-vertex_closest[0])))
                
                if(theta1 > 0):
                    relative_theta = block_theta - abs(theta1)
                else:
                    relative_theta = np.deg2rad(90) - block_theta - abs(theta1)

                angle_to_pick_up = 15
                if (abs(relative_theta) < lowest_theta) \
                        and ((abs(relative_theta) \
                        < np.deg2rad(angle_to_pick_up)) \
                        or (abs(relative_theta) \
                        > np.deg2rad(90-angle_to_pick_up))):
                    lowest_theta = abs(relative_theta)
                    lowest_theta_transform = robot_to_tag_1
        if is_moving == 1:
            pub_valid_cube.publish(0)
        else:
            if lowest_theta != lowest_theta_set:
                current_pose = Pose()
                current_pose.position.x = lowest_theta_transform[0][3]
                current_pose.position.y =  lowest_theta_transform[1][3]
                current_pose.position.z = lowest_theta_transform[2][3]
                if previous_pose != current_pose:
                    i=0
                    while i < 3:
                        pub_valid_cube.publish(1)
                        pub.publish(current_pose)
                        state_pub.publish(7)
                        i+=1
                else:
                    pub_valid_cube.publish(0)  
            else:
                pub_valid_cube.publish(0)
            previous_pose = msg
    elif state==7:
        pub.publish(current_pose)
        
    elif state==4:
        transform_array.append(np.array([[0, 1, 0, 0.0],
                                         [1, 0, 0, -0.03],
                                         [0, 0, -1, 0.22],
                                         [0, 0, 0, 1]]))
        x_dist = -0.035
        y_dist = -0.014
        z_dist = -0.038
        robot_base_to_robot_joint = np.array([[1, 0, 0, x_dist],
                                              [0, 1, 0, y_dist],
                                              [0, 0, 1, z_dist],
                                              [0, 0, 0, 1]])

        camera_to_tag_1 = transform_array[0]
        robot_base_to_tag_1 = np.matmul(np.linalg.inv(camera_to_robot_base),\
            camera_to_tag_1)
        robot_to_tag_1 = np.matmul(np.linalg.inv(robot_base_to_robot_joint),\
            robot_base_to_tag_1)

        msg = Pose()
        msg.position.x = robot_to_tag_1[0][3]
        msg.position.y =  robot_to_tag_1[1][3]
        msg.position.z = robot_to_tag_1[2][3]
        pub.publish(msg)
    
    else:
        pub_valid_cube.publish(0)

    camera_to_tag_1 = np.zeros((4,4))
    
if __name__=='__main__':
    global pub
    global listener
    global sub_transform,tfBuffer,pub_ignore,state_pub,pub_valid_cube,state
    state = 0
    #loop_count = 0
    sub_transform = np.array([[1,0,0,0],[0,1,0,-0.3],[0,0,1,0.5],[0,0,0,1]])

    pub_ignore = rospy.Publisher('ignore_fiducials',msg.String, queue_size=10)
    pub = rospy.Publisher('desired_position', Pose, queue_size=10)
    pub_valid_cube = rospy.Publisher("is_valid_cube", msg.Int8, queue_size=10)
    state_pub= rospy.Publisher('state', msg.Int16, queue_size=10)

    sub = rospy.Subscriber('state', msg.Int16, store_state)
    sub = rospy.Subscriber('is_moving', msg.Int8, store_is_moving)
    vertex_sub = rospy.Subscriber("fiducial_vertices", FiducialArray , \
        store_vertex)
    sub_point = rospy.Subscriber('fiducial_transforms',FiducialTransformArray, \
        T_C2R)

    rospy.init_node('tag_selector')
    rospy.spin()  