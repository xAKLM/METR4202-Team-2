import numpy as np
import rospy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from inverse_kinematics import invKin
from inverse_kinematics import analytical
from joint_state_publisher import JointPublisher
from tfListener import CameraGet

from gpiozero import AngularServo
from gpiozero.pins.rpigpio import RPiGPIOFactory
from gpiozero import Device

Device.pin_factory = RPiGPIOFactory()
from time import sleep
import rospy

if __name__ == '__main__':
    rospy.init_node('hey')
    js = JointPublisher()
    cg = CameraGet()
    while not rospy.is_shutdown():
        T = cg.get_transform('fiducial_33', 'fiducial_7')
        print(T)
    
