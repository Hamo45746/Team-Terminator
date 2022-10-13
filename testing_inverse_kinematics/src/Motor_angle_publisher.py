"""
This script publishes a joint states to the dynamixel controller.
This is in response to a desired end effector point (angle assumed to be 0 atm)
"""


# Always need this
import rospy

# Import message types
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# inverse kinematics stuff
import numpy as np
import math as m

print('running')

# my functions
def give_Px(theta_2_guess,theta_3_guess,L1,L2,L3):
    return L1*m.cos(-theta_2_guess - theta_3_guess) + L2*m.cos(-theta_3_guess) + L3*m.cos(0)

def give_Py(theta_2_guess,theta_3_guess,L1,L2,L3):
    return L1*m.sin(-theta_2_guess - theta_3_guess) + L2*m.sin(-theta_3_guess) + L3*m.sin(0)


def generate_derivative_matrix(L1,L2,theta_2_guess,theta_3_guess):
    # | df_1/dtheta_2 , df_1/dtheta_3 |
    # | df_2/dtheta_2 , df_1/dtheta_3 |
    return np.array([
        [((theta_2_guess + theta_3_guess)*L1*m.sin(-theta_2_guess - theta_3_guess) + 0 + 0), ((theta_2_guess + theta_3_guess)*L1*m.sin(-theta_2_guess - theta_3_guess) + theta_3_guess*L2*m.sin(-theta_3_guess) + 0)],
        [((-theta_2_guess - theta_3_guess)*L1*m.cos(-theta_2_guess - theta_3_guess) + 0 + 0) , ((-theta_2_guess - theta_3_guess)*L1*m.cos(-theta_2_guess - theta_3_guess) - theta_3_guess*L2*m.cos(-theta_3_guess) + 0)]
        ])

# Your inverse kinematics function
# This one doesn't actually do it though...
def inverse_kinematics(pose: Pose) -> JointState:
    global pub
    print('recieved')
    # insert inverse kinematics here
    L1 = 0.115
    L2 = 0.095
    L3 = 0.07
    robot_origin = [0 , 0.07]

    Px_desired = 0.15
    Py_desired = 0.02
    threshhold = 0.01 # for error
    phi_desired = 0 * (np.pi/180)


    theta_1_guess = 50 * (np.pi/180) # 50 these worked:
    theta_2_guess = 15 * (np.pi/180) # 15
    theta_3_guess = 15 * (np.pi/180) # 15

    # using theta_1 = -theta_2_guess - theta_3_guess (with phi = 0) we can simplify the problem:
    Px_current = L1*m.cos(-theta_2_guess - theta_3_guess) + L2*m.cos(-theta_3_guess) + L3*m.cos(0)
    Py_current = L1*m.sin(-theta_2_guess - theta_3_guess) + L2*m.sin(-theta_3_guess) + L3*m.sin(0)
    
    error = 100
    i = 1
    while error > threshhold:

        if i == 1:
            theta_2 = theta_2_guess
            theta_3 = theta_3_guess
        
        theta_vec = np.array([
            [theta_2],
            [theta_3]
            ])

        g_theta = np.array([
            [give_Px(theta_2,theta_3,L1,L2,L3)],
            [give_Py(theta_2,theta_3,L1,L2,L3)]
            ])

        derivative_matrix = generate_derivative_matrix(L1,L2,theta_2,theta_3)
        invderivative = np.linalg.inv(derivative_matrix)

        theta_next = theta_vec - np.matmul(invderivative , g_theta)
        theta_2_last = theta_next[0][0]
        theta_3_last = theta_next[1][0]

        theta_2 = theta_2_last
        theta_3 = theta_3_last


        theta_1 = -theta_2_last - theta_3_last

        # Px_current is f_1
        # Py_current is f_2

        theta1_abs =  theta_1                     # 65 * np.pi/180
        theta2_relative = theta_2                 # -45 * np.pi/180 # Note that theta2 is relative to theta1
        theta2_abs = theta1_abs + theta2_relative
        theta3_relative = theta_3                 # -30 * np.pi/180 # Note that theta2 is relative to theta1
        theta3_abs = theta2_abs + theta3_relative

        
        # generate line 1 arrays
        L1_start = robot_origin
        L1_end = [L1_start[0] + L1*np.cos(theta1_abs),L1_start[1] + L1*np.sin(theta1_abs)]
        x1 = np.linspace(L1_start[0],L1_end[0],num=50)
        y1 = np.linspace(L1_start[1],L1_end[1],num=50)           
        

        # generate line 2 arrays
        L2_start = [L1_end[0],L1_end[1]]
        L2_end = [L2_start[0] + L2*np.cos(theta2_abs), L2_start[1] + L2*np.sin(theta2_abs)]
        x2 = np.linspace(L2_start[0],L2_end[0],num=50)
        y2 = np.linspace(L2_start[1],L2_end[1],num=50)

        # generate line 3 arrays
        L3_start = [L2_end[0],L2_end[1]]
        L3_end = [L3_start[0] + L3*np.cos(theta3_abs), L3_start[1] + L3*np.sin(theta3_abs)]
        x3 = np.linspace(L3_start[0],L3_end[0],num=50)
        y3 = np.linspace(L3_start[1],L3_end[1],num=50)


        check_negatives = 0
        for num in y1:
            if num < 0:
                check_negatives = 1
        for num in y2:
            if num < 0:
                check_negatives = 1
        for num in y3:
            if num < 0:
                check_negatives = 1   
        if (check_negatives == 1) or (L1_end[0] < L1_start[0]):
            continue

        print(i)

        error = m.sqrt((Px_desired - L3_end[0])**2 + (Py_desired - L3_end[1])**2)
        i = i+1


    # FIND THE LEAST FACTOR ANGLE

    if theta1_abs > 0:
        theta1_abs = theta1_abs%(2*np.pi)
        if theta1_abs > np.pi:
            theta1_abs = -1*(2*np.pi - theta1_abs)
    else:
        theta1_abs = (1*theta1_abs)%(-2*np.pi)

    if theta2_relative > 0:
        theta2_relative = theta2_relative%(2*np.pi)
        if theta2_relative > np.pi:
            theta2_relative = -1*(2*np.pi - theta2_relative)
    else:
        theta2_relative = (theta2_relative)%(-2*np.pi)
        if theta2_relative < -np.pi:
            theta2_relative = 1*(2*np.pi - theta2_relative)

    if theta3_relative > 0:
        theta3_relative = (theta3_relative) % (2*np.pi)
        if theta3_relative > np.pi:
            theta3_relative = -1*(2*np.pi - theta3_relative)
    else:
        theta3_relative = (theta3_relative)%(-2*np.pi)
        if theta3_relative < -np.pi:
            theta3_relative = 1*(2*np.pi + theta3_relative)

    

    # CORRECT THETA1 TO THE VERTICAL
    if theta1_abs > np.pi/2: # > 90deg
        theta1_abs = theta1_abs - np.pi/2
    elif theta1_abs < np.pi/2:
        theta1_abs = np.pi/2 - theta1_abs
    
    print('thetas no conversion to robot',theta1_abs,theta2_relative,theta3_relative)


    # define the msg to be published
    msg = JointState(
            # Set header with current time
            header=Header(stamp=rospy.Time.now()),
            # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
            name=['joint_1','joint_2', 'joint_3', 'joint_4']
        )
        # Funny code
    msg.position = [
        0,
        -theta1_abs,
        theta2_abs,
        -theta3_abs
        ]   # THIS DOESNT INCLUDE HORIZONTAL  

    rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
    pub.publish(msg)

'''
# Funny code
def dummy_joint_states() -> JointState:
    # Create message of type JointState
    msg = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
    )
    # Funny code
    msg.position = [
        random.uniform(-1.5, 1.5),
        random.uniform(-1.5, 1.5),
        random.uniform(-1.5, 1.5),
        random.uniform(-1.5, 1.5)
    ]
    return msg
'''

def main():
    global pub
    # Create publisher
    pub = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )

    # Create subscriber
    sub = rospy.Subscriber(
        'desired_position', # Topic name
        Pose, # Message type
        inverse_kinematics # Callback function (required)
    )

    # Initialise node with any node name
    rospy.init_node('inverse_kinematics')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()


