import numpy as np
import rospy
from tf2_msgs.msg import TFMessage
import tf2_ros
import modern_robotics as mr
from tf.transformations import quaternion_matrix 
from tf.transformations import euler_from_quaternion
import tf
import math

from fiducial_msgs.msg import FiducialTransformArray
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from fiducial_msgs.msg import FiducialArray
from color_detector import ColorDetector

#OFFSET VALUES
X_OFFSET = 0.039 
Y_OFFSET = 0 #0.004
Z_OFFSET = 0

class CameraGet:
    """
    Handles the information given from the camera

    """
    def __init__(self, cd, baseFrame):
        """ 
        cd          -   type ColorDetector used to detect colors
        baseFrame   -   type String used to distinguish base fiducial ID
        """
        self.cd = cd
        self.baseFrame = baseFrame

        """ Information to Store """
        self.vert = None
        self.data = None
        self.desiredFrame = None
        self.pixelCheck = 1
        self.prevRotY = None
        self.prevT = None
        self.isRotate = False
        self.red = 0
        self.green = 0
        self.blue = 0
        self.yellow = 0

        """ Subscribers """
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.Subscriber('fiducial_vertices', FiducialArray, self.get_vert)
        rospy.Subscriber('/ximea_cam/image_raw', Image, self.get_data_pixel)
        rospy.Subscriber('/tf', FiducialTransformArray, self.get_tf)
        rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.check_moving)
        self.rate = rospy.Rate(50)

    
    def is_rotate(self):
        """ returns the rotation state of the fiducial """
        while self.isRotate == True:
            print("moving")
            pass

    def check_moving(self, data):
        """ 
        Checks if the fiducials are moving

        data    -   type FiducialTransformArray used to get the information of fiducials

        """
        if (self.prevT == None):

            for i in data.transforms:
                #assuming baseFrame will be a 2 digit aruco tag
                if (i.fiducial_id ==  int(self.baseFrame[-2:])):
                    continue
                
                #store the fiducial_id and its rotation
                self.prevT = i.fiducial_id
                r = i.transform.rotation
                rpy = euler_from_quaternion([r.x, r.y, r.z, r.w])
                newRotY= rpy[2]
                self.prevRotY = rpy[2]

        else:

            for i in data.transforms:
                #find the same fiducial_id
                if (i.fiducial_id == self.prevT):
                    #compare the rotations
                    r = i.transform.rotation
                    rpy = euler_from_quaternion([r.x, r.y, r.z, r.w])
                    newRotY= rpy[2]
                    #if the difference is less than 0.1
                    if (math.isclose(newRotY, self.prevRotY, abs_tol=0.1)):
                        self.isRotate = False
                    else:
                        self.isRotate = True
            #reset the revT and prevRotY
            self.prevT = None
            self.prevRotY = None

    def get_tf(self, data):
        """

        Obtains the transformation matrix of the fiducial

        data    -   type FiducialTransformArray used to obtain transformation

        """
        #if the desiredFrame doesnt already exist
        if (self.desiredFrame == None):
            for i in data.transforms:
                if (i.child_frame_id != self.baseFrame):
                    self.desiredFrame = i.child_frame_id
                    self.rotY = i.transform.rotation.y
                    break



    def get_vert(self, vert):
        """

        Obtains the fiducial vertices of desiredFrame 

        vert    -   type FiducialArray used to get vertices

        """
        if (self.desiredFrame != None):
            for i in vert.fiducials:
                l = len(self.desiredFrame) - 9 #9 is length of "fiducial_"
                if (i.fiducial_id == int(self.desiredFrame[-l:])):
                    self.vert = i
                    self.pixelCheck = 1
                    #setting this lets us get_pixel_data once
                    break

    
    def is_frame(self):
        """ Checks if a desiredFrame exists """
        while not rospy.is_shutdown():
            print(self.desiredFrame)
            if self.desiredFrame != None:
                return

    def get_data_pixel(self, image):
        """ 
        grabs the color of the data pixel

        image   -   type Image the pixels of the camera

        """
        if (self.vert != None and self.pixelCheck == 1):
            self.pixelCheck = 0
            data = image.data
            x0 = self.vert.x0
            y0 = self.vert.y0
            val = 1
            val2 = 1
            count = 0

            while not rospy.is_shutdown():
                #index one way until we find a color thats not white or black 
                index = int((1280 * (round(y0)) + (round(x0) - val)) * 3)
                color = self.cd.detect_color(np.array([int(data[index]), int(data[index + 1]), int(data[index + 2])]).astype(np.uint8))
                if color < 4:
                    break
                val = val + 1
                count = count + 1
                if count > 50:
                    #try indexing the other way
                    index = int((1280 * (round(y0)) + (round(x0) + val2)) * 3)
                    color = self.cd.detect_color(np.array([int(data[index]), int(data[index + 1]), int(data[index + 2])]).astype(np.uint8))
                    if color < 4:
                        break
                    val = val + 1
                    count = count + 1
                    if count > 100:
                        break

            if (color == 0):
                self.red += 1
                #"RED"
            if (color == 1):
                self.green += 1
                #"GREEN"
            if (color == 2):
                self.blue += 1
                # "BLUE"
            if (color == 3):
                self.yellow += 1
                #"YELLOW"
 

    def get_data(self):
        """
        This was used to test what was happening within the callbacks

        """
        rospy.spin()

    
    def get_color(self):
        """

        Lists the 4 colors (RGBY) into a tuple and finds the color with highest occurrence
        

        Returns:        color with highest occurrence

        """
        initial = 0
        allColors = (self.red, self.green, self.blue, self.yellow)
        for (i, colors) in enumerate(allColors):
            if (colors >= initial):
                initial = colors
                color = i

        if (color == 0):
            return "RED"
        if (color == 1):
            return "GREEN"
        if (color == 2):
            return "BLUE"
        if (color == 3):
            return "YELLOW"


    def get_transform(self):
        """

        Gets the transformation from baseFrame to desiredFrame using tf2listener

        Returns:        transformation matrix of desiredFrame

        """
        if (self.desiredFrame == None):
            return
        
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform(self.baseFrame, self.desiredFrame, rospy.Time())
                self.tfBuffer.clear()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue

            rp = (trans.transform)
            """ Store the quarternion_matrix and x,y,z positions in a transformation matrix """
            matrix = (quaternion_matrix([rp.rotation.x, rp.rotation.y, rp.rotation.z, rp.rotation.w]))
            matrix[0,3] = rp.translation.x + X_OFFSET
            matrix[1,3] = rp.translation.y + Y_OFFSET
            matrix[2,3] = rp.translation.z

            """ Convert quarternion to euler angles and store them on the 4th row of matrix"""
            rpy = euler_from_quaternion([rp.rotation.x, rp.rotation.y, rp.rotation.z, rp.rotation.w])
            print(rpy)
            matrix[3,0] = rpy[0]
            matrix[3,1] = rpy[1]
            matrix[3,2] = rpy[2]

            print(matrix)   
            return matrix

    def reset(self):
        """ resets the desiredFrame, data, vert and rotY to None """
        self.desiredFrame = None
        self.data = None
        self.vert = None
        self.rotY = None

    def reset_color(self):
        """ resets the color occurences back to (0, 0, 0, 0) """
        self.red = 0
        self.blue = 0
        self.green = 0
        self.yellow = 0


if __name__ == '__main__':

    """ CODE ONLY USED TO TEST CALLBACK FUNCTIONS """

    rospy.init_node('hey')
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
    cs = CameraGet(color_detector, "fiducial_12")
    cs.get_data()
    
