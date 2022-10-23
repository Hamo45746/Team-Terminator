#!/usr/bin/python3


from telnetlib import X3PAD
import rospy

from geometry_msgs.msg import Point

#import tf2_ros
from tf2_msgs.msg import TFMessage
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
import numpy as np
import std_msgs.msg as msg
import tf2_ros
from geometry_msgs.msg import Pose
import geometry_msgs.msg
from fiducial_msgs.msg import FiducialArray



def trans(t: Transform):

    # Get homegeneous transformation matrix from Transform (camera to aruco tag)

    w = t.Quaternion.w

    x = t.Quaternion.x

    y = t.Quaternion.y

    z = t.Quaternion.z

    T_C2A = np.array([[2*((w^2)+(x^2))-1, 2*(x*y-w*z), 2*(x*z+w*y), t.Vector3.x],

    [2*(x*y+w*z), 2*((w^2)+(y^2))-1, 2*(y*z-w*x), t.Vector3.y],

    [2*(x*z-w*y), 2*(y*z+w*x), 2*((w^2)+(z^2))-1, t.Vector3.z],

    [0, 0, 0, 1]])

 

    # Measured distances between centre of aruco tag and centre of robot base (initially set to 0)

    # Assuming exactly zero rotation between aruco tag and robot base (will need to be very precise or



    # Multiply transformation matrix by translation from aruco tag to robot base

    #T_A2R = np.array([1,0,0,x_dist],[0,1,0,y_dist],[0,0,1,z_dist],[0,0,0,1]) 
    #return np.matmul(T_C2A, T_A2R)

def store_is_moving(i):
    global is_moving
    is_moving = i.data

def store_state(i):
    global state
    state = i.data

def store_vertex(i):
    global vertex
    vertex = i

# id for the camera
#info_camera.header.frame_id = std::string("ximea_") + serial;

previous_pose = Pose()

loop_count = 0
def T_C2R(p):
    # maybe use fiducial vertices to get colour?????????????
    global camera_to_robot_base # idk why this is global
    global pub
    global loop_count
    global listener
    global sub_transform
    global tfBuffer
    global msg,state,vertex,pub_ignore,camera_to_robot,previous_pose,state_pub,ismoving_pub,pub_valid_cube,current_pose
    print('converter outside of if statements: state=',state)
    
    #print('hi1')
    transform_array = []
    k=0
    
    #camera_to_robot = np.array([[],[],[],[]])
    if (state==0):
        print('in state 0 converter loop','\n\n\n\n')
    elif (p.transforms != []) and (state==2):
        print('in converter loop')
        print(p.transforms)
        lowest_theta_set = 10000
        lowest_theta = lowest_theta_set
        for i,j in enumerate(p.transforms):
            #transform_array.append(j)
            #w = j.transform.rotation.w
            #x = j.transform.rotation.x
            #y = j.transform.rotation.y
            #z = j.transform.rotation.z
            #transform_array.append(np.array([
            #    [2*((w**2)+(x**2))-1, 2*(x*y-w*z), 2*(x*z+w*y), j.transform.translation.x],
            #    [2*(x*y+w*z), 2*((w**2)+(y**2))-1, 2*(y*z-w*x), j.transform.translation.y],
            #    [2*(x*z-w*y), 2*(y*z+w*x), 2*((w**2)+(z**2))-1, j.transform.translation.z],
            #    [ 0, 0, 0, 1]
            #]))

            j_id = j.fiducial_id
            if (j_id != 1) and (j_id!=2):
                '''
                # for tag id = 1:
                x_dist = -0.027
                y_dist = -0.0
                z_dist = -0.042
                '''
                # for tag id = 2:
                x_dist = -0.014
                y_dist = -0.03
                z_dist = -0.042
                cam_to_robot_x = 0
                cam_to_robot_y = -0.203
                cam_to_robot_z = 0.376

                camera_to_robot =  np.array([
                    [0 ,1 , 0, cam_to_robot_x],
                    [1 ,0 , 0, cam_to_robot_y],
                    [0 ,0 , -1, cam_to_robot_z],
                    [ 0, 0, 0, 1]
                ])
                #print('cam to robot',camera_to_robot)
                camera_to_tag_1 = np.array([
                    [0 ,1 , 0, j.transform.translation.x],
                    [1 ,0 , 0, j.transform.translation.y],
                    [0 ,0 , -1, j.transform.translation.z],
                    [ 0, 0, 0, 1]
                ])
                #print(camera_to_tag_1)
                
                print(camera_to_robot)
                robot_to_tag_1 = np.matmul(np.linalg.inv(camera_to_robot),camera_to_tag_1)
                print('robot to tag state 2',robot_to_tag_1)

                desired_y = robot_to_tag_1[1][3]
                desired_x = robot_to_tag_1[0][3]
                theta1 = np.arctan(desired_y/desired_x)
                
                #new code
                vertex_0 = [0,0]
                vertex_1 = [0,0]
                vertex_2 = [0,0]
                vertex_3 = [0,0]
                vertex_closest = [0,0]
                vertex_right = [0,0]

                vertex_0[0] = vertex.fiducials[0].x0
                vertex_0[1] = vertex.fiducials[0].y0
                vertex_1[0] = vertex.fiducials[0].x1
                vertex_1[1] = vertex.fiducials[0].y1
                vertex_2[0] = vertex.fiducials[0].x2
                vertex_2[1] = vertex.fiducials[0].y2
                vertex_3[0] = vertex.fiducials[0].x3
                vertex_3[1] = vertex.fiducials[0].y3

                vertex_closest[1] = min(vertex_0[1],vertex_1[1],vertex_2[1],vertex_3[1])
                if(vertex_closest[1] == vertex_0[1]):
                    vertex_closest[0] = vertex_0[0]
                elif(vertex_closest[1] == vertex_1[1]):
                    vertex_closest[0] = vertex_1[0]
                elif(vertex_closest[1] == vertex_2[1]):
                    vertex_closest[0] = vertex_2[0]
                elif(vertex_closest[1] == vertex_3[1]):
                    vertex_closest[0] = vertex_3[0]

                vertex_right[0] = min(vertex_0[0],vertex_1[0],vertex_2[0],vertex_3[0])
                if(vertex_right[0] == vertex_0[0]):
                    vertex_right[1] = vertex_0[1]
                elif(vertex_right[0] == vertex_1[0]):
                    vertex_right[1] = vertex_1[1]
                elif(vertex_right[0] == vertex_2[0]):
                    vertex_right[1] = vertex_2[1]
                elif(vertex_right[0] == vertex_3[0]):
                    vertex_right[1] = vertex_3[1]               

                block_theta = abs(np.arctan((vertex_right[1]-vertex_closest[1]) / (vertex_right[0]-vertex_closest[0])))

                # if(theta1 < 0):
                #     block_theta = np.deg2rad(90) - block_theta

                #new code finish

                #old code start
                # x0 = vertex.fiducials[0].x0
                # y0 = vertex.fiducials[0].y0
                # x1 = vertex.fiducials[0].x1
                # y1 = vertex.fiducials[0].y1
                # x2 = vertex.fiducials[0].x2
                # y2 = vertex.fiducials[0].y2

                # block_theta = min(abs(np.arctan((y1-y0) / (x1-x0))),abs(np.arctan((y2-y1) / (x2-x1))))
                #old code finish
                
                if(theta1 > 0):
                    relative_theta = block_theta - abs(theta1)
                else:
                    relative_theta = np.deg2rad(90) - block_theta - abs(theta1)

                # if(abs(relative_theta) > np.deg2rad(45)):
                #     if(relative_theta) < 0:
                #         relative_theta += np.deg2rad(45)
                #     else:
                #         relative_theta -= np.deg2rad(45)

                print("theta1: ",np.rad2deg(theta1))
                print("relative_theta: ",np.rad2deg(relative_theta))
                print("block_theta: ",np.rad2deg(block_theta))

                print('in loop here')
                print('angles,theta1,block',theta1,block_theta,relative_theta)
                angle_to_pick_up = 15
                if (abs(relative_theta) < lowest_theta) and ((abs(relative_theta) < np.rad2deg(angle_to_pick_up)) or (abs(relative_theta) > np.rad2deg(90-angle_to_pick_up))):
                    lowest_theta = abs(relative_theta)
                    lowest_theta_transform = robot_to_tag_1
                
                # need to see if the angle is greater than a threshold #################################################################################
        print('\n\n\n\n',lowest_theta,'\n\n\n\n')
        
        if is_moving == 1:
            pub_valid_cube.publish(0)
            print("in is moving if")
        else:
            if lowest_theta != lowest_theta_set:
                current_pose = Pose()
                current_pose.position.x = lowest_theta_transform[0][3]
                current_pose.position.y =  lowest_theta_transform[1][3]
                current_pose.position.z = lowest_theta_transform[2][3]
                print('pose',current_pose,previous_pose)
                if previous_pose != current_pose:
                    i=0
                    while i < 3:
                        pub_valid_cube.publish(1)
                        #pub.publish(current_pose)
                        state_pub.publish(7)
                        i+=1
                else:
                    print('in second else')
                    pub_valid_cube.publish(0)
                    
            else:
                print('in else')
                pub_valid_cube.publish(0)
            previous_pose = msg

    elif state==7:
        pub.publish(current_pose)

    elif state==4:
        transform_array.append(np.array([
            [0 ,1 , 0, 0.0],
            [1 ,0 , 0, -0.03],
            [0 ,0 ,-1, 0.22],
            [ 0, 0, 0, 1]
        ]))
        x_dist = -0.035
        y_dist = -0.014
        z_dist = -0.038
        robot_base_to_robot_joint = np.array([
                [1,0 , 0, x_dist],
                [0,1 , 0, y_dist],
                [0 ,0 , 1, z_dist],
                [ 0, 0, 0, 1]
        ])
        print(state)
        #print('cam to robot',camera_to_robot)
        camera_to_tag_1 = transform_array[0]
        #print('camner to tag',camera_to_tag_1)
        ##camera_to_tag_2 = transform_array[1]
        #camera_to_robot = 
        robot_base_to_tag_1 = np.matmul(np.linalg.inv(camera_to_robot_base),camera_to_tag_1)
        robot_to_tag_1 = np.matmul(np.linalg.inv(robot_base_to_robot_joint),robot_base_to_tag_1)
        #print(robot_to_tag_1)


        msg = Pose()
        msg.position.x = robot_to_tag_1[0][3]
        msg.position.y =  robot_to_tag_1[1][3]
        msg.position.z = robot_to_tag_1[2][3]
        pub.publish(msg)
    else:
        print('empty  transform, not in state 0, 2, or 4')
        pub_valid_cube.publish(0)

    camera_to_tag_1 = np.zeros((4,4))

    #this is how to do it with tfmessage
    #print(p.transforms[0].child_frame_id,p.transforms[0].transform.translation)
    '''
    try:
        print('hi2')
        #trans = tfBuffer.lookup_transform("fiducial_12","ximea_31704051")
        #print(trans)
    except:
        d=1
    #'''


    
    #robot_frame_p = np.matmul(sub_transform, np.array([p.transforms.transform.translation.x],[p.transform.translation.y],[p.transform.translation.z],[1]))



    #msg = Point(robot_frame_p[0],robot_frame_p[1],robot_frame_p[2])
    #print(msg)
    #pub.publish(msg)


def main():
    global pub
    global listener
    global sub_transform,tfBuffer,pub_ignore,state_pub,pub_valid_cube
    
    #tfBuffer = tf2_ros.Buffer()
    #listener = tf2_ros.TransformListener(tfBuffer)
    sub_transform = np.array([[1,0,0,0],[0,1,0,-0.3],[0,0,1,0.5],[0,0,0,1]])
    pub_ignore = rospy.Publisher('ignore_fiducials',msg.String, queue_size=10)

    pub = rospy.Publisher('desired_position', Pose, queue_size=10)
    #sub_transform = rospy.Subscriber('tf', Transform, trans)
    sub = rospy.Subscriber(
        'state', # Topic name
        msg.Int16, # Message type
        store_state # Callback function (required)
    )

    pub_valid_cube = rospy.Publisher("is_valid_cube", msg.Int8, queue_size=10)

    state_pub= rospy.Publisher(
        'state', # Topic name
        msg.Int16, # Message type
        queue_size=10 # Topic size (optional)
    )

    sub = rospy.Subscriber(
        'is_moving', # Topic name
        msg.Int8, # Message type
        store_is_moving # Callback function (required)
    )

    vertex_sub = rospy.Subscriber("fiducial_vertices", FiducialArray , store_vertex)
    #sub_point = rospy.Subscriber('tf',TFMessage, T_C2R)#replace placeholders with actual topics
    sub_point = rospy.Subscriber('fiducial_transforms',FiducialTransformArray, T_C2R)#replace placeholders with actual topics
    rospy.init_node('frame_converter')
    rospy.spin()


if __name__=='__main__':
    loop_count = 0
    main()          