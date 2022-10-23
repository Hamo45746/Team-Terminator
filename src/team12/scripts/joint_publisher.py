#!/usr/bin/python3

# Import message types
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import std_msgs.msg as msg

# inverse kinematics stuff
import numpy as np
import math as m

def desired_locations():
    '''
    Inputs: N/A
    Returns: list[desired_end_angle, desireed_z, desired_distance]
    - desired_end_angle: Desired end effector angle of the robot
    - desired_z: the desired height of the end effector angle
    - desired_distance: the desired distance of the end effector from the \
        robot origin
    '''

    deg_0_threshhold = 0.19
    deg_45_thresh = 0.16
    desired_distance = calculate_distance(desired_position) ###########
    
    # Calculate the parameters to be returned based on thresholds
    if desired_distance < deg_45_thresh:
        desired_end_angle = np.deg2rad(-65)
        desired_z = -0.01 # final values for demo z = -0.01 ####################
    elif (deg_45_thresh <= desired_distance <= deg_0_threshhold):
        desired_end_angle = np.deg2rad(-65)
        desired_z = -0.01 # final values for demo z = -0.01 ###################
    elif (desired_distance > deg_0_threshhold):
        desired_end_angle = np.deg2rad(-5)
        desired_z = 0.02# final values for demo z = 0.02 ######################
    elif (desired_distance > deg_0_threshhold+0.03):
        desired_end_angle = np.deg2rad(-5)
        desired_z = 0.045 # final values for demo z = 0.03 ####################
    return [desired_end_angle, desired_z, desired_distance]

def colour_checker():
    '''
    inputs: colour (string): the string received from the topic 'colour'
    returns: theta1(float): the requiured theta1 angle for the recieved colour
    '''
    if colour == "B":
        return -0.2
    elif colour == "R":
        return 0.2
    elif colour == "G":
        return -1.33
    elif colour == "Y":
        return 1.33
    else:
        pub_state.publish(-1)
        return 0

def store_position(i):
    '''
    Stores the required end effector position
    inputs: i (Pose), required pose for end effector
    outputs: N/A
    '''
    global desired_position
    desired_position = i

def store_colour(i):
    '''
    Stores the colour detected by the camera
    inputs: i (String), the detected colour
    outputs: N/A
    '''
    global colour
    colour = i.data

def store_state(i):
    '''
    Stores the current state
    inputs: i (Int8), the current state
    outputs: N/A
    '''
    global state
    state = i.data

def store_valid_cube(i):
    '''
    Stores the variable determining if the cube is a valid pickup-position
    inputs: i (Int), if the current cube is a valid pickup.
    outputs: N/A
    '''
    global is_valid_cube
    is_valid_cube = i.data

def calculate_distance(pose):
    desired_x = pose.position.x
    desired_y = pose.position.y  
    return np.sqrt(desired_x**2 + desired_y**2) 

def generate_message(thetalist):
    msg = JointState(
            header=Header(stamp=rospy.Time.now()),
            name=['joint_1','joint_2', 'joint_3', 'joint_4']
        )

    msg.position = [thetalist[0], thetalist[1], thetalist[2], thetalist[3]]
 
    return msg

def publish_message(state, message, iterations):
    i=0
    while (i < iterations):
        pub_joint.publish(message)
        if(state == 0):
            pass  
        elif(state == -1):
            pub_joint.publish(message)
        elif(state == 1):
            pub_wait_state.publish(message)
        elif(state == 7):
            pub_grab_state.publish(message)
        elif(state == 4):
            pub_colour_state.publish(message)
        elif(state == 5):
            pub_drop_state.publish(message)
        elif(state == 8):
            pub_inter_state.publish(message)
        i += 1

def inverse_kinematics(desired_end_angle,pose,desired_z,desired_distance, \
        link_lengths):
    desired_x = pose.position.x
    desired_y = pose.position.y
    L1 = link_lengths[0]
    L2 = link_lengths[1]
    L3 = link_lengths[2]
    L4 = link_lengths[3]

    theta1 = np.arctan(desired_y/desired_x)

    offset = 0.01
    desired_distance_actual = desired_distance - L4*np.cos(desired_end_angle) \
        - offset*np.sin(desired_end_angle)
    desired_elevation_actual = desired_z - L4*np.sin(desired_end_angle) \
        - L1 - offset*np.cos(desired_end_angle)  

    elbow_down = -1 # ie elbow up
    costheta3 = (desired_distance_actual**2 + desired_elevation_actual**2 - \
        L2**2 - L3**2) / (2*L2*L3)
    theta3 = m.atan2(elbow_down*m.sqrt(1 - costheta3**2),costheta3)
    theta2 = m.atan2(desired_elevation_actual,desired_distance_actual) - \
        np.arctan2(L2*np.sin(theta3),L2 + L3*np.cos(theta3))
    
    theta4 =  desired_end_angle - theta2 - theta3

    # CORRECT THETA2 TO THE VERTICAL
    if theta2 > np.pi/2: # > 90deg
        theta2 = theta2 - np.pi/2
    elif theta2 < np.pi/2:
        theta2 = np.pi/2 - theta2
    
    # Account for Dynamixel rotation directions
    theta2=-theta2
    theta4=-theta4

    return generate_message([theta1,theta2,theta3,theta4])

def joint_angle_publisher(pose):
    # pose in this case is a  joint states type, not pose type
    global pub_joint,colour,state,pub_state,desired_position,is_valid_cube, \
        state_7_msg,pub_inter_state

    link_lengths = [0.055,0.118,0.094,0.1] #[L1, L2, L3, L4]
    
    if (state == 0):
        msg = generate_message([0,0,0,0])
        publish_message(0,msg,1)
    elif (state==-1): # waiting state
        publish_message(-1,state_7_msg,50)
    elif (state==1): # intermediate state
        thetalist = [0, 0.72, -1.44, 0.112]
        msg = generate_message(thetalist)
        publish_message(1,msg,50)
    elif (state == 7):        
        desired_locations_list = desired_locations()

        desired_end_angle = desired_locations_list[0]
        desired_z = desired_locations_list[1]
        desired_distance = desired_locations_list[2]

        state_7_msg = inverse_kinematics(desired_end_angle,desired_position, \
            desired_z, desired_distance,link_lengths)

        if is_valid_cube == 1:
            publish_message(7,state_7_msg,30)
    elif (state == 4): # colour state
        msg = generate_message([0.03,-0.711,-0.4244,-1.279])
        publish_message(4,msg,30)
    elif (state == 5):
        thetalist = [0, 0.48, 1.54, -0.92]
        thetalist[0] = colour_checker()
        msg = generate_message(thetalist)  
        publish_message(5,msg,30)
    elif (state == 8):
       
        theta1 = state_7_msg.position[0]
        theta2 = state_7_msg.position[1]
        theta3 = state_7_msg.position[2]
        theta4 = state_7_msg.position[3]

        state_8_msg = generate_message([theta1,theta2+1,theta3,theta4])
        publish_message(8,state_8_msg,30)

if __name__ == '__main__':
    global pub_colour_state,pub_joint,pub_drop_state,pub_grab_state, \
        pub_wait_state,pub_state,pub_inter_state

    # Create publisher
    pub_joint = rospy.Publisher('desired_joint_states', JointState, \
        queue_size=50)

    pub_wait_state = rospy.Publisher('desired_wait_states', JointState, \
        queue_size=10)

    pub_grab_state = rospy.Publisher('desired_grab_states', JointState, \
        queue_size=10)

    pub_colour_state = rospy.Publisher('desired_colour_states', JointState, \
        queue_size=10)

    pub_drop_state = rospy.Publisher('desired_drop_states', JointState, \
        queue_size=10)
  
    pub_state = rospy.Publisher('state', msg.Int16, queue_size=10)

    pub_inter_state = rospy.Publisher('inter_state', JointState, queue_size=10)

    sub = rospy.Subscriber('desired_position', Pose, store_position)

    sub = rospy.Subscriber('joint_states', JointState, joint_angle_publisher)
    
    sub = rospy.Subscriber('state', msg.Int16, store_state)

    sub = rospy.Subscriber('is_valid_cube', msg.Int8, store_valid_cube)

    sub = rospy.Subscriber('colour', msg.String, store_colour)

    rospy.init_node('joint_publisher')
    rospy.spin()
