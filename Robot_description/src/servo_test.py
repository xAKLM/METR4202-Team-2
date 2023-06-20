from gpiozero import AngularServo
#from gpiozero.pins.rpigpio.RPiGPIOFactory import NativeFactory
from gpiozero.pins.rpigpio import RPiGPIOFactory
from gpiozero import Device

Device.pin_factory = RPiGPIOFactory()
from time import sleep
import rospy

if __name__ == "__main__":

    servo = AngularServo(12, initial_angle=-10, min_angle=-10, max_angle=5)
    while(1):
        servo.angle = -6
        print("zero angle")
        rospy.sleep(3)
        servo.angle = -4
        print("closed")
        rospy.sleep(2)
        print("open")
        servo.angle = -8
        rospy.sleep(2)
    while(1):
        rospy.sleep(2)
        servo.max()
        servo.angle = 0
        print("max")
        rospy.sleep(2)
        servo.angle = 0
        print("min")
        servo.min()




