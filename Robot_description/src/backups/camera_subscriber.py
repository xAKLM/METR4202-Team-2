import numpy as np
import rospy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from inverse_kinematics import invKin
from inverse_kinematics import analytical
from fiducial_msgs.msg import FiducialArray
import tf
from color_detector import ColorDetector

bgr_r = np.array([0,0,255]).astype(np.uint8)
bgr_g = np.array([0,255,0]).astype(np.uint8)
bgr_b = np.array([255,0,0]).astype(np.uint8)
bgr_y = np.array([0,255,255]).astype(np.uint8)


class cameraSubscriber():
    def __init__(self, cd):
        self.basePos = None
        self.desiredPos = None
        self.vert = None
        self.data = None
        rospy.Subscriber('fiducial_vertices', FiducialArray, self.get_vert)
        rospy.Subscriber('/ximea_cam/image_raw', Image, self.get_data_pixel)
        self.cd = cd


    #stores the fiducial vertices arrays
    def get_vert(self, vert):
        if (self.vert == None and len(vert.fiducials) != 0):
            self.vert = vert.fiducials[0]

    def old_code(self, image):
        if (self.vert != None):
            data = image.data
            x0 = self.vert.x0
            y0 = self.vert.y0
            index = int((1280 * (round(y0) - 5) + (round(x0) + 2)) * 3)
            print('NEW COLOUR:')
            print(int(data[index]), int(data[index + 1]), int(data[index + 2]), self.vert.fiducial_id)
            print('-----------------------------')
            x0 = self.vert.x1
            y0 = self.vert.y1
            index = int((1280 * (round(y0) - 5) + (round(x0) + 2)) * 3)
            print(int(data[index]), int(data[index + 1]), int(data[index + 2]), self.vert.fiducial_id)
            print('-----------------------------')
            x0 = self.vert.x2
            y0 = self.vert.y2
            index = int((1280 * (round(y0) - 5) + (round(x0) + 2)) * 3)
            print(int(data[index]), int(data[index + 1]), int(data[index + 2]), self.vert.fiducial_id)
            print('-----------------------------')
            x0 = self.vert.x3
            y0 = self.vert.y3
            index = int((1280 * (round(y0) - 5) + (round(x0) + 2)) * 3)
            print(int(data[index]), int(data[index + 1]), int(data[index + 2]), self.vert.fiducial_id)
            print('----------------------------')

            self.vert = None
            rospy.sleep(1)

    def get_data_pixel(self, image):
        print("nope")
        if (self.vert != None):
            print("yep")

            data = image.data
            x0 = self.vert.x0
            y0 = self.vert.y0
            val = 1
            count = 0
            while not rospy.is_shutdown():
                index = int((1280 * (round(y0)) + (round(x0) - val)) * 3)
                #image_raw gives gbr (convert gbr to bgr format)
                color = self.cd.detect_color(np.array([int(data[index]), int(data[index + 1]), int(data[index + 2])]).astype(np.uint8))
                print(color)
                if color < 4:
                    break
                val = val + 1
                count = count + 1
            if (color == 0):
                print("RED")
            if (color == 1):
                print("GREEN")
            if (color == 2):
                print("BLUE")
            if (color == 3):
                print("YELLOW")
            print('didnt find')



        
    
    def get_data(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('hey')
    current_dir = "/home/metr4202/catkin_ws/src/Robot_description/src"

    try:
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
    except FileNotFoundError:
        print("Could not load color.config file")
        print("Setting to default values:")
        bgr_r = np.array([0, 0, 255]).astype(np.uint8)
        bgr_g = np.array([0, 255, 0]).astype(np.uint8)
        bgr_b = np.array([255, 0, 0]).astype(np.uint8)
        bgr_y = np.array([0, 255, 255]).astype(np.uint8)
    color_detector = ColorDetector(bgr_r, bgr_g, bgr_b, bgr_y)
    cs = cameraSubscriber(color_detector)
    cs.get_data()









