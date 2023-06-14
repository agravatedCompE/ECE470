#!/usr/bin/env python
import sys
import copy
import time
import rospy

import numpy as np
from lab5_header import *
from lab5_func import *
from blob_search import *

block_attached = False

def grip_check():
    global SPIN_RATE
    loop_rate = rospy.Rate(SPIN_RATE)
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)
    if(not block_attached):
        rospy.loginfo("No block, error")
        gripper(pub_command, loop_rate, suction_off)
        return 2
    return 1


def suction_callback(msg):
    global suction_on
    global suction_off
    global block_attached
    block_attached = msg.DIGIN


# Position for UR3 not blocking the camera
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]

# Store world coordinates of green and yellow blocks
xw_yw_G = []
xw_yw_Y = []

# global variable
dest_xw_yw_G = np.array([   [0.198, -0.097],
                            [0.198, -0.155],
                            [0.140, -0.096],
                            [0.141, -0.154] ])

dest_xw_yw_Y = np.array([   [0.317, -0.099],
                            [0.314, -0.160],
                            [0.257, -0.097],
                            [0.258, -0.156] ])


# 20Hz
SPIN_RATE = 20

# UR3 home location
home = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)
thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
digital_in_0 = 0
analog_in_0 = 0.0
suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False
image_shape_define = False
"""
Whenever ur3/gripper_input publishes info this callback function is
called.
"""
'''def input_callback(msg):
    global digital_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1 # Only look at least significant bit,meaning index 0'''
"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):
    global thetas
    global current_position
    global current_position_set
    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]
    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]
    current_position_set = True
"""
Function to control the suction cup on/off
"""
def gripper(pub_cmd, loop_rate, io_0):
    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position
    error = 0
    spin_count = 0
    at_goal = 0
    current_io_0 = io_0
    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

while(at_goal == 0):
    if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
        abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
        abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
        abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
        abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
        abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):
        #rospy.loginfo("Goal is reached!")
        at_goal = 1
    loop_rate.sleep()

    if(spin_count > SPIN_RATE*5):
        pub_cmd.publish(driver_msg)
        rospy.loginfo("Just published again driver_msg")
        spin_count = 0
    spin_count = spin_count + 1
    return error


"""
Move robot arm from one position to another
"""
def move_arm(pub_cmd, loop_rate, dest, vel, accel):
    global thetas
    global SPIN_RATE
    error = 0
    spin_count = 0
    at_goal = 0
    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)
    loop_rate.sleep()
    while(at_goal == 0):
        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):
            at_goal = 1
        #rospy.loginfo("Goal is reached!")
        loop_rate.sleep()
        if(spin_count > SPIN_RATE*5):
            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0
        spin_count = spin_count + 1
    return error


def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel):
    """
    start_xw_yw_zw: where to pick up a block in global coordinates
    target_xw_yw_zw: where to place the block in global coordinates
    """

    print(lab_invk(0.2, 0.2, 0, 0))

    gripper(pub_cmd, loop_rate, suction_off)

    #move above start
    angles = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], 0.100, 0)
    print(angles)
    move_arm(pub_cmd, loop_rate, angles, 4.0, 4.0)

    #move onto start
    gripper(pub_cmd, loop_rate, suction_on)
    angles = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], 0.030, 0)
    print(angles)
    move_arm(pub_cmd, loop_rate, angles, 4.0, 4.0)

    #wait
    time.sleep(.1)
    if(grip_check()==2):
        return
    
    #move above start
    angles = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], 0.100, 0)
    print(angles)
    move_arm(pub_cmd, loop_rate, angles, 4.0, 4.0)

    #move to target above
    angles = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], 0.100, 0)
    print(angles)
    move_arm(pub_cmd, loop_rate, angles, 4.0, 4.0)

    #move down to target
    angles = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], 0.030, 0)
    print(angles)
    move_arm(pub_cmd, loop_rate, angles, 4.0, 4.0)

    #drop block
    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(.1)

    #move abaove target
    angles = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], 0.100, 0)
    print(angles)
    move_arm(pub_cmd, loop_rate, angles, 4.0, 4.0)
    error = 0

    
    return error



class ImageConverter:
    def __init__(self, SPIN_RATE):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        self.loop_rate = rospy.Rate(SPIN_RATE)
        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
            print("ROS is shutdown!")

    def image_callback(self, data):
        global xw_yw_G # store found green blocks in this list
        global xw_yw_Y # store found yellow blocks in this list
        try:
            # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        cv_image = cv2.flip(raw_image, -1)
        cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

        # need to call blob_search() function to find centers of green blocks
        # and yellow blocks, and store the centers in xw_yw_G & xw_yw_Y respectively.
        # If no blocks are found for a particular color, return an empty list,
        # to xw_yw_G or xw_yw_Y.

        # xw_yw_G & xw_yw_Y are in global coordinates, which means
        # do coordinate transformation in the blob_search() function, from
        # the image frame to the global world frame.
        xw_yw_G = blob_search(cv_image, "green")
        #xw_yw_Y = blob_search(cv_image, "yellow")
        xw_yw_Y = blob_search(cv_image, "orange")
        #print(xw_yw_O)


"""
Program run from here
"""
def main():
    global go_away
    global xw_yw_R
    global xw_yw_G
    # global variable1
    # global variable2
    # Initialize ROS node
    rospy.init_node('lab5node')
    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)
    # Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    #sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)
    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, suction_callback)
    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)
    vel = 4.0
    accel = 4.0
    move_arm(pub_command, loop_rate, go_away, vel, accel)
    ic = ImageConverter(SPIN_RATE)
    time.sleep(5)
    """
    use the found xw_yw_G, xw_yw_Y to move the blocks
    correspondingly. need to call move_block(pub_command, loop_rate, start_xw_yw_zw,
    target_xw_yw_zw, vel, accel)
    """
    #save globals to locals
    xw_yw_Gl = xw_yw_G
    xw_yw_Yl = xw_yw_Y
    #for each block in green, move from blob coords to green drop offs
    i = 0
    for g in xw_yw_Gl:
        move_block(pub_command, loop_rate, g, dest_xw_yw_G[i], vel, accel)
        i = i+1
    #for each block in yellow, move from blob coords to yellow drop offs
    i = 0
    for y in xw_yw_Yl:
        move_block(pub_command, loop_rate, y, dest_xw_yw_Y[i], vel, accel)
        i = i+1

    move_arm(pub_command, loop_rate, go_away, vel, accel)
    
    #move_arm(pub_command, loop_rate, go_away, vel, accel)
    rospy.loginfo("Task Completed!")
    print("Use Ctrl+C to exit program")
    rospy.spin()



if __name__ == '__main__':
    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass

    