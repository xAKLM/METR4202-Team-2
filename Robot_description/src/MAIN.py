import numpy as np
import rospy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from inverse_kinematics import invKin
from inverse_kinematics import analytical
from JOINT_STATE_PUBLISHER import JointPublisher
from color_detector import ColorDetector
from TFLISTENER import CameraGet
import pigpio
import time

#CONSTANTS
servoPin = 12
servoPins = 32
BASE_ID = "fiducial_12"

#set-up pigpio servo
pi = pigpio.pi()
pi.set_mode(servoPin, pigpio.OUTPUT)


def get_color_detector():
    """

    Creates an instance of ColorDetector()

    Returns     -   type ColorDetector cd, used to help detect rgb colors


    """

    #from uqmvale6 github code 

    current_dir = "/home/metr4202/catkin_ws/src/Robot_description/src"
    file = open(current_dir + "/colors.config", 'r')
    i = 0
    for line in file:
        entries = line.split(' ')
        values = [int(x) for x in entries]
        if i == 0:
            bgr_r = np.array([values[0], values[1], values[2]]).astype(np.uint8)
        if i == 1:
            bgr_g = np.array([values[0], values[1], values[2]]).astype(np.uint8)
        if i == 2:
            bgr_b = np.array([values[0], values[1], values[2]]).astype(np.uint8)
        if i == 3:
            bgr_y = np.array([values[0], values[1], values[2]]).astype(np.uint8)
        i += 1
    color_detector = ColorDetector(bgr_r, bgr_g, bgr_b, bgr_y)
    return color_detector

def servo_drop():
    """ puts servo in DROP postion """
    pi.set_servo_pulsewidth(servoPin, 1170)

def servo_grab():
    """ puts servo in GRAB postion """
    pi.set_servo_pulsewidth(servoPin, 1500)

def servo_fake():
    """ puts servo in FAKE postion (look dont ask any questions ok)"""
    pi.set_servo_pulsewidth(servoPin, 1400)

def do_work(cd, cg, js):
    """

    Main functionality code, ideally to be looped infinitely

    cd  -   type ColorDetector() used to help detect RGB colors
    cg  -   type CameraGet() used to get information from cameras
    js  -   type JointPublisher() used to publish joint information


    """
    
    """ Start off in FAKE pos and go HOME"""
    servo_drop()
    js.go_home(False)
    rospy.sleep(3)
    cg.reset()

    """ check if the blocks are ROTATING """
    cg.is_rotate()

    """ check if a desiredPos has been stored and if so, store it to T"""
    cg.is_frame()
    T = cg.get_transform()
    
    """ test if the desiredPos is reachable with ik """
    if (js.reachable == False):
        js.reachable = True
        return

    """ go to the desiredPos but start at the top """
    js.go_to_pos(T, False)
    rospy.sleep(2)
    
    """ go to the desiredPos but drop down """
    js.go_to_pos(T, True)
    rospy.sleep(0.8)
    servo_drop()
    rospy.sleep(0.3)
    
    """ grab the block and pull back up """
    servo_grab()
    rospy.sleep(2)
    js.go_to_pos(T, False)
    rospy.sleep(2)

    """ get the blocks color and determine give it BLUE if its None """
    color = cg.get_color()
    print(color)
    if (color == None):
        color = "BLUE"

    """ reset the desiredPos and go to pads """
    cg.reset()
    js.go_to_pad(color, False)
    rospy.sleep(3)

    """ start at the top of the pad, drop and go back up """
    js.go_to_pad(color, True)
    rospy.sleep(1)
    servo_drop()
    rospy.sleep(1)
    js.go_to_pad(color, False)
    rospy.sleep(1)
    servo_fake()
    rospy.sleep(1)

    """ reset for next loop """
    cg.reset()
    cg.reset_color()

def go_to_colors(js):
    """ Just goes to the color pads (USED FOR TESTING) """
    js.go_to_pad("GREEN", False)
    print("GREEN")
    rospy.sleep(3)
    js.go_to_pad("RED", False)
    print("RED")
    rospy.sleep(3)
    js.go_to_pad("BLUE", False)
    print("BLUE")
    rospy.sleep(3)
    js.go_to_pad("YELLOW", False)
    rospy.sleep(3)

def main():

    rospy.init_node('hey')
    cd = get_color_detector()
    cg = CameraGet(cd, BASE_ID)
    js = JointPublisher()

    while not rospy.is_shutdown():
        do_work(cd, cg, js)
        #T = cg.get_transform()
        #print(T)

if __name__ == '__main__':
    main()
