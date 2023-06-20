import numpy as np
import rospy
from sensor_msgs.msg import JointState
import modern_robotics as mr
import tf


class JointPublisher:
    """ Class used to publish to /desired_joint_states """
    
    def __init__(self):
        """ Initialise the publisher """
        self.pub = rospy.Publisher('desired_joint_states', JointState, queue_size=10)

        
        self.reachable = True
        self.message = JointState()
        self.rate = rospy.Rate(50)
        rospy.sleep(3)
    
        self.basePos = None
        self.desiredpos = None

    def tweak_joints(self, thetaList):
        """ used to wrap the joint values (from uqmvale6 github) """
        thetaList = -((np.pi - thetaList) % (2.0 * np.pi) - np.pi)
        return thetaList

    def analytical(self, T, down):
        """ 

        Calculates the analytical solution for the SCARA ARM

        T       -   type tf used to get x,y and rotY values
        down    -   type boolean used to specify whether to extend down (when True)

        Returns     list of joint values to publish

        """
        #length of the arms
        L2 = 0.10
        L1 = 0.11827

        (rotation,position) = mr.TransToRp(T)

        x = position[0] 
        y = position[1]
        z = position[2]
        
        #refer to derivation somewhere else
        betaArg = (-(x**2 + y**2) +  (L1**2 + L2**2)) / (2 * L1 * L2)
        gammaArg = (L1**2 +  x**2 + y**2 - L2**2) / (2 * L1 * np.sqrt(x**2 + y**2))
        hype = x**2 + y**2
        length = L1 + L2

        #checks if solution is reachable
        if (-1 > betaArg or betaArg > 1 ):
            print("cannot reach")
            self.reachable = False
            return


        beta = np.arccos(betaArg)
        gamma = np.arccos(gammaArg)

        
        theta_1 = np.arctan2(y,x) - gamma
        theta_2 = np.pi - beta 

        #if user specified down, go down
        if (down == True):
            theta_3 = 1.8 

        else:
            #otherwise go up
            theta_3 = 5

        #end effector angle (depends on the angle of theta_1 and theta_2)
        theta_4 = T[3,1] - (theta_1 + theta_2)

        #wrap the theta_4 value to dynamixel limits
        if (theta_4 > 2.6):
            theta_4 = - (theta_4 - np.pi)
            print('adjusted')
        elif (theta_4 < -2.6):
            theta_4 = - (theta_4 + np.pi)
            print('adjusted')

        joints = np.array([theta_1, theta_2, theta_3, theta_4])
        print(joints)
        return joints 


    def publish_frame(self, T, down):
        """

        Publish frame T to /desired_joint_states

        T       -   type transformation matrix used to get desired pose
        down    -   type boolean, true if arm is intended to go down

        """

        thetaList = self.analytical(T, down)
        if (self.reachable == False):
            return
        #print('calculated analytical')
        #thetaList = self.tweak_joints(thetaList)
        #print('tweaked joints')

        self.make_message(thetaList)
        print('message made, attempting to publish')
        self.pub.publish(self.message)
        #print('message published :)')
        


    def make_message(self, position):
        """ 

        Makes a message for given joint angles

        position    -   type tuple used to make message to publish to /desired_joint_states

        """
        names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        velocity = [1.8,1.8,4,2]
        effort = [0,0,0,0]
    
        self.message.name = names
        self.message.position = position
        self.message.effort = effort
        self.message.velocity = velocity
        

    def go_to_pos(self, T, down):
        """ 
        Go to a position specified by T matrix

        T   -   type transformation matrix used to get desired pose

        """
        self.publish_frame(T, down)


    def go_home(self, down):
        """ Go to home position """
        HOME = np.array([[1, 0, 0, 0],
                         [0, 1, 0, -0.21827],
                         [0, 0, 1, .07138],
                         [0, 0, 0,       1]])
        self.publish_frame(HOME, down)
    
    def go_to_pad(self, number, down):
        """ 
        Go to a loading zone, publishes the transform to a loading zone


        number  -   type string (sry) loading zone color
        down    -   type boolean go down?

        """
        #NOTE: ALL HARDCODED IN
        PAD_ONE = np.array([[1, 0, 0, -0.03],
                         [0, 1, 0, 0.15],
                         [0, 0, 1,  .07138],
                         [0, 0, 0,       1]])

        PAD_TWO = np.array([[1, 0, 0, -0.17],
                         [0, 1, 0, 0.04],
                         [0, 0, 1,  0.218],
                         [0, 0, 0,       1]])

        PAD_THREE = np.array([[1, 0, 0, -0.10],
                         [0, 1, 0, -0.13],
                         [0, 0, 1,       0.07],
                         [0, 0, 0,       1]])

        PAD_FOUR = np.array([[1, 0, 0,  -0.03],
                         [0, 1, 0,      -0.16],
                         [0, 0, 1,      0.218],
                         [0, 0, 0,       1]])

        if (number == "RED"):
            self.publish_frame(PAD_ONE, down)
            return
        if (number == "YELLOW"):
            self.publish_frame(PAD_TWO, down)
            return 
        if (number == "GREEN"):
            #had trouble with going to this pos with ik, so hardcoded joints in
            if (down == True):
                t3 =  1.5 

            else:
                t3 = 5
            thetaList = [-2, -1.25, t3, 0]
            self.make_message(thetaList)
            self.pub.publish(self.message)
            return
        if (number == "BLUE"):
            self.publish_frame(PAD_FOUR, down)
            return
