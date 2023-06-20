import numpy as np
import rospy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from inverse_kinematics import invKin
from inverse_kinematics import analytical
from JOINT_STATE_PUBLISHER import JointPublisher
from TFLISTENER import CameraGet


import pigpio
import time

servoPin = 12

pi = pigpio.pi()
pi.set_mode(servoPin, pigpio.OUTPUT)

def do_work(js, cg, pad):
    js.go_home(False)
    rospy.sleep(4)
    T = cg.get_transform('fiducial_42', 'fiducial_2')
    print(T)
    js.go_to_pos(T, False)
    rospy.sleep(2)
    js.go_to_pos(T, True)
    rospy.sleep(2)
    grab()
    rospy.sleep(2) 
    js.go_to_pos(T, False)
    rospy.sleep(2)
    js.go_to_pad(pad, False)
    rospy.sleep(2)
    drop()
    rospy.sleep(2) 
    js.go_to_pad(pad, False)

def servo_drop():
    pi.set_servo_pulsewidth(servoPin, 1200)

def servo_grab():
    pi.set_servo_pulsewidth(servoPin, 1500)

        

def main():
    rospy.init_node('big_boi')

    js = JointPublisher()
    cg = CameraGet()
    pad = 1
    js.go_home(False)
    rospy.sleep(3)
    servo_drop()

    #while not rospy.is_shutdown():
    #    if (pad == 4):
    #        pad = 1
    #    do_work(js,cg,pad)
    #    pad = pad + 1
        #drop()
        #rospy.sleep(2)
        #grab()
        #rospy.sleep(2)
    #js.go_to_pos(T, False)
    #print(T)
    T  = cg.get_transform('fiducial_9', 'fiducial_6')
    print(T)


    js.go_to_pos(T, False)
    rospy.sleep(2)
    js.go_to_pos(T, True)
    rospy.sleep(2)
    servo_grab()
    rospy.sleep(2)
    js.go_to_pos(T, False)
    rospy.sleep(1)

    js.go_to_pad(4, False)
    rospy.sleep(2)
    servo_drop()
    pi.stop()


    
    

    
    

if __name__ == '__main__':
    main()





