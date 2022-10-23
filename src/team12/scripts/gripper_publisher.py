#!/usr/bin/python3
# Always need this
import rospy
import pigpio
# Import message types
import std_msgs.msg as msg

rpi = pigpio.pi()
rpi.set_mode(18, pigpio.OUTPUT)

def publish(state, iterations):
    i = 0
    while (i < iterations):
        pub.publish(state)
        count+=1


def move_gripper(i): 
    #1000 is the closed position
    #1500 is the grip box position
    #2000 is the open position
    grab = 1410
    open = 2000
    
    if i.data == 3:
        rospy.sleep(0.5)
        rpi.set_servo_pulsewidth(18,grab)
        rospy.sleep(0.7)
        publish(8,40)
    elif (i.data == 6):
        rospy.sleep(0.15)
        rpi.set_servo_pulsewidth(18,open)
        publish(1,70)
    elif i.data == 1:
        rospy.sleep(0.15)
        rpi.set_servo_pulsewidth(18,open)

if __name__ == '__main__':
    global pub
    # Create publisher
    pub = rospy.Publisher('state', msg.Int16, queue_size=10)

    # Create subscriber
    sub = rospy.Subscriber('state', msg.Int16, move_gripper)

    rospy.init_node('gripper')
    rospy.spin()
