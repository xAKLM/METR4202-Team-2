import numpy as np
import rospy
from sensor_msgs.msg import JointState
from inverse_kinematics import invKin
from inverse_kinematics import analytical
import modern_robotics as mr
import tf


class JointPublisher:
    
    def __init__(self):
        self.pub = rospy.Publisher('desired_joint_states', JointState, queue_size=10)

        
        self.message = JointState()
        self.rate = rospy.Rate(10)
        rospy.sleep(3)
    
        self.basePos = None
        self.desiredpos = None

    def tweak_joints(self, thetaList):
        #CHANGE FOR MORE ACCURACY
        #radius = 0.0225
        #thetaList[2] = -thetaList[2] / radius
        #WRAP
        thetaList = -((np.pi - thetaList) % (2.0 * np.pi) - np.pi)
        return thetaList

    def analytical(self, T, down):
        L2 = 0.10
        L1 = 0.11827
        radius = 0.0225

        (rotation,position) = mr.TransToRp(T)
        print(position)

        x = position[0] 
        y = position[1]
        z = position[2]

        betaArg = (-(x**2 + y**2) +  (L1**2 + L2**2)) / (2 * L1 * L2)
        gammaArg = (L1**2 +  x**2 + y**2 - L2**2) / (2 * L1 * np.sqrt(x**2 + y**2))
        hype = x**2 + y**2
        length = L1 + L2
        if (-1 > betaArg or betaArg > 1 ):
            print("cannot reach")
            return


        beta = np.arccos(betaArg)
        gamma = np.arccos(gammaArg)

        
        theta_1 = np.arctan2(y,x) - gamma
        theta_2 = np.pi - beta 
        if (down == True):
            theta_3 =  1.5 #(0.08-0.07138)/radius 

        else:
            theta_3 = 5


        theta_4 = T[3,1] - (theta_1 + theta_2)
        if (theta_4 > 2.6):
            theta_4 = - (theta_4 - np.pi)
        elif (theta_4 < -2.6):
            theta_4 = - (theta_4 + np.pi)

        joints = np.array([theta_1, theta_2, theta_3, theta_4])
        print(joints)
        return joints 


    def publish_frame(self, T, down):

        thetaList = self.analytical(T, down)
        print('calculated analytical')
        #thetaList = self.tweak_joints(thetaList)
        print('tweaked joints')
        self.make_message(thetaList)
        print('message made, attempting to publish')
        self.pub.publish(self.message)
        print('message published :)')
        


    def make_message(self, position):
        names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        velocity = [1.8,1.8,4,2]
        effort = [0,0,0,0]
    
        self.message.name = names
        self.message.position = position
        self.message.effort = effort
        self.message.velocity = velocity
        

    def go_to_pos(self, T, down):
        self.publish_frame(T, down)


    def go_home(self, down):
        HOME = np.array([[1, 0, 0, 0],
                         [0, 1, 0, -0.21827],
                         [0, 0, 1, .07138],
                         [0, 0, 0,       1]])
        self.publish_frame(HOME, down)
    
    def go_to_pad(self, number, down):
        PAD_ONE = np.array([[1, 0, 0, -0.03],
                         [0, 1, 0, 0.15],
                         [0, 0, 1,  .07138],
                         [0, 0, 0,       1]])

        PAD_TWO = np.array([[1, 0, 0, -0.17],
                         [0, 1, 0, 0.04],
                         [0, 0, 1,  0.218],
                         [0, 0, 0,       1]])

        PAD_THREE = np.array([[1, 0, 0, -0.17],
                         [0, 1, 0, -0.07],
                         [0, 0, 1,       0.07],
                         [0, 0, 0,       1]])

        PAD_FOUR = np.array([[1, 0, 0,  -0.03],
                         [0, 1, 0,      -0.16],
                         [0, 0, 1,      0.218],
                         [0, 0, 0,       1]])
        if (number == 1):
            self.publish_frame(PAD_ONE, down)
            return
        if (number == 2):
            self.publish_frame(PAD_TWO, down)
            return 
        if (number == 3):
            self.publish_frame(PAD_THREE, down)
            return
        if (number == 4):
            self.publish_frame(PAD_FOUR, down)
            return

    def clear(self):
        self.basePos = None
        self.desiredPos = None






    


