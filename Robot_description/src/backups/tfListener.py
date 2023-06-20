import numpy as np
import rospy
from tf2_msgs.msg import TFMessage
from inverse_kinematics import invKin
from inverse_kinematics import analytical
import tf2_ros
import modern_robotics as mr
from tf.transformations import quaternion_matrix 
from tf.transformations import euler_from_quaternion


class CameraGet:

    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)


        self.rate = rospy.Rate(50)

    def get_transform(self, frame_1, frame_2):

        X_OFFSET = 0.039 
        Y_OFFSET = -0.005
        Z_OFFSET = 0
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform(frame_1, frame_2, rospy.Time())
                self.tfBuffer.clear()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue
            rp = (trans.transform)
            matrix = (quaternion_matrix([rp.rotation.x, rp.rotation.y, rp.rotation.z, rp.rotation.w]))
            matrix[0,3] = rp.translation.x + X_OFFSET
            matrix[1,3] = rp.translation.y + Y_OFFSET
            matrix[2,3] = rp.translation.z



            rpy = euler_from_quaternion([rp.rotation.x, rp.rotation.y, rp.rotation.z, rp.rotation.w])
            print(rpy)
            matrix[3,0] = rpy[0]
            matrix[3,1] = rpy[1]
            matrix[3,2] = rpy[2]
            zrotate = rpy[2]
            print("THIS WORD")
            print(matrix[3,1])




            

            print(matrix)
            
            return matrix

    def print_T(self, frame_1, frame_2):
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform(frame_1, frame_2, rospy.Time())
                print(trans)
                self.tfBuffer.clear()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue

    




