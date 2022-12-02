#!/usr/bin/python3
#This is for screen capture

import cv2
import numpy as np
from PIL import Image
from time import time
from mss import mss
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import OccupancyGrid

originalMap = OccupancyGrid()
received = False
originalMapCopy = OccupancyGrid()

class humanDetector:
    def __init__(self):
        # Create mask for red color, a low level and a high level range
        self.low_red = np.array([0, 50, 50])
        self.high_red = np.array([10, 255, 255])

    def detect(self, frame):
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Create masks with color ranges
        mask = cv2.inRange(hsv_img, self.low_red, self.high_red)
        # Find Contours of the selection between those ranges of color
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
        box = (0, 0, 0, 0)
        for cnt in contours:
            (x, y, w, h) = cv2.boundingRect(cnt)
            box = (x, y, x + w, y + h)
            break

        return box #Returns the coordinates.

class otherObjectsDetector:
    def __init__(self):
        # Create mask for red color, a low level and a high level range
        self.low_red = np.array([0, 50, 50])
        self.high_red = np.array([10, 255, 255])

    def detect(self, frame):
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create masks with color ranges
        mask = cv2.inRange(hsv_img, self.low_red, self.high_red)
        # Find Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

        box = (0, 0, 0, 0)
        for cnt in contours:
            (x, y, w, h) = cv2.boundingRect(cnt)
            box = (x, y, x + w, y + h)
            break

        return box #Returns the coordinates.


def findMap(frame):
    # Create mask for black
    low_black = np.array([0, 0, 0])
    high_black = np.array([180, 255, 40])
    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create masks with color ranges
    mask = cv2.inRange(hsv_img, low_black, high_black)

    # Find Contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
    box = (0, 0, 0, 0)
    for cnt in contours:
        (x, y, w, h) = cv2.boundingRect(cnt)
        box = (x, y, x + w, y + h)
        break

    return box #Returns the coordinates.
    
class KalmanFilter:
    kf = cv2.KalmanFilter(4, 2) #this creates the kalman filter with a measurement matrix of 2 and state of 4 
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32) #measurement matrix
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32) #transition


    def predict(self, coordX, coordY):
        ''' This function estimates the position of the object'''
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]]) #This is the data we send
        self.kf.correct(measured) #Apply the correction step
        predicted = self.kf.predict() #predict 
        x, y = int(predicted[0]), int(predicted[1])
        return x, y

def mapCallback(information):
    global originalMap, received
    originalMap = information #saves the gridmap

    mon = {'top': 540, 'left':80, 'width':500, 'height':500} #location of the camera (window)
    sct = mss()
    begin_time = time()
    sct_img = sct.grab(mon) #take screenshot of the window
    
    #some conversions of images to bgr code
    img = Image.frombytes('RGB', (sct_img.size.width, sct_img.size.height), sct_img.rgb)
    img_bgr = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)

    # Load detector
    redDetection = humanDetector()

    # Load Kalman filter to predict the trajectory
    kalman = KalmanFilter()

   # Publishing subscribers
    publishVisual = rospy.Publisher("visualReal", PoseStamped, queue_size=1)
    publishPredicted = rospy.Publisher("visualReal", PoseStamped, queue_size=1)
    publishMap = rospy.Publisher("mapTwo", OccupancyGrid, queue_size=1)

    #Camera capture and 
    while True:
        begin_time = time()
        sct_img = sct.grab(mon)
        img = Image.frombytes('RGB', (sct_img.size.width, sct_img.size.height), sct_img.rgb)
        img_bgr = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)

        
        #print('This frame takes {} seconds.'.format(time()-begin_time))
        
        cv2.imshow('test', np.array(img_bgr)) #Shows the test window

        if received == False: #For the original initialization moves the windwos into another position
            cv2.moveWindow("test", 800, 700)

         
        map_box = findMap(img_bgr) #finds the map corners and stores
        x, y, x2, y2 = map_box #map cordinates
        img_bgr = img_bgr[y:y2,x:x2] #delimits the map

        img_bgr = cv2.resize(img_bgr, (500,500), interpolation=cv2.INTER_AREA) #resizes the map

        red_box = redDetection.detect(img_bgr)
        x, y, x2, y2 = red_box

        cx = int((x + x2) / 2)
        cy = int((y + y2) / 2)

        cx = (cx - 250)*0.05
        cy = ((cy*-1)+250)*0.05

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = cx
        goal_pose.pose.position.y = cy

        publishVisual.publish(goal_pose)


        if received == False:
            received = True


        key = cv2.waitKey(60)
        if key == 27:
            break

def main():
    global received

    rospy.init_node("Vision")

    mapSubscriber = rospy.Subscriber("map", OccupancyGrid, mapCallback)

    rospy.spin()

    
main()

