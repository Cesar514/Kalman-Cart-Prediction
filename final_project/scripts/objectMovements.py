#!/usr/bin/python3
#This is for screen capture

import cv2
import numpy as np
from PIL import Image
from time import time
from mss import mss
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

originalMap = OccupancyGrid()

class humanDetector:
    def __init__(self):
        # Create mask for red color
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

        return box

class otherObjectsDetector:
    def __init__(self):
        # Create mask for red color
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

        return box


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

    return box
    
class KalmanFilter:
    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)


    def predict(self, coordX, coordY):
        ''' This function estimates the position of the object'''
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        self.kf.correct(measured)
        predicted = self.kf.predict()
        x, y = int(predicted[0]), int(predicted[1])
        return x, y

def mapCallback(information):
    originalMap = information
    #rospy.loginfo(originalMap)
    return


def main():
    
    mon = {'top': 540, 'left':80, 'width':500, 'height':500}

    sct = mss()
    begin_time = time()
    sct_img = sct.grab(mon)
    img = Image.frombytes('RGB', (sct_img.size.width, sct_img.size.height), sct_img.rgb)
    img_bgr = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
    #capture = cv2.VideoCapture(img_bgr)

    # Load detector
    redDetection = humanDetector()

    # Load Kalman filter to predict the trajectory
    kalman = KalmanFilter()

    mapSubscriber = rospy.Subscriber("map", OccupancyGrid, mapCallback)

    publishVisual = rospy.Publisher("visualReal", PoseStamped, queue_size=1)
    publishPredicted = rospy.Publisher("visualReal", PoseStamped, queue_size=1)
    publishMap = rospy.Publisher("mapTwo", OccupancyGrid, queue_size=1)

    while True:
        begin_time = time()
        sct_img = sct.grab(mon)
        img = Image.frombytes('RGB', (sct_img.size.width, sct_img.size.height), sct_img.rgb)
        img_bgr = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)

        
        #print('This frame takes {} seconds.'.format(time()-begin_time))
        
        cv2.imshow('test', np.array(img_bgr))

        map_box = findMap(img_bgr)
        x, y, x2, y2 = map_box
        img_bgr = img_bgr[y:y2,x:x2]


        

        
        img_bgr = cv2.resize(img_bgr, (500,500), interpolation=cv2.INTER_AREA)


        low_black = np.array([0, 0, 0])
        high_black = np.array([132, 132, 132])

        hsv_img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        masked = cv2.inRange(hsv_img,low_black,high_black)

        masked = cv2.bitwise_not(masked)

        cv2.imshow("Masked", masked)
        
        map = OccupancyGrid()

        #masked = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
        (thresh, masked) = cv2.threshold(masked, 132, 255, cv2.THRESH_BINARY)
        masked = masked.astype(np.uint8)
        newMask = []

        for number in masked:
            for inside in number:
                if inside == 255:
                    inside = 100
                newMask.append(inside)
        
        id = 0
        for number in originalMap.data:
            rospy.loginfo(number)
            """ if number == -1:
                newMask[id] = -1
            elif number == 100:
                newMask[id] = 100
            else:
                newMask[id] = newMask[id]
            id = id + 1 """


        #rospy.loginfo(newMask)

        map = originalMap
        map.data = newMask

        publishMap.publish(map)

        

        red_box = redDetection.detect(img_bgr)
        x, y, x2, y2 = red_box



        cx = int((x + x2) / 2)
        cy = int((y + y2) / 2)

        predicted = kalman.predict(cx, cy)

        for i in range(1):
            predicted = kalman.predict(predicted[0], predicted[1])

        


        #cv2.rectangle(img_bgr, (x, y), (x2, y2), (255, 0, 0), 4)
        cv2.circle(img_bgr, (cx, cy), 2, (0, 255, 0), 1)

        cv2.circle(img_bgr, (predicted[0], predicted[1]), 8, (255, 0, 0), 4)


        cx = (cx - 250)*0.05
        cy = ((cy*-1)+250)*0.05

        

        goal_pose = PoseStamped()
        goal_pose.pose.position.x = cx
        goal_pose.pose.position.y = cy

        publishVisual.publish(goal_pose)

        cx = (predicted[0] - 250)*0.05
        cy = ((predicted[1]*-1)+250)*0.05

        goal_pose.pose.position.x = cx
        goal_pose.pose.position.y = cy
        
        publishPredicted.publish(goal_pose)



        #rospy.loginfo("This is X: " + str(cx) + "This is Y: " + str(cy))
        #rospy.loginfo("This is Kalman X: " + str(predicted[0]) + "This is Kalman Y: " + str(predicted[1]))

        cv2.imshow("Frame", img_bgr)
        key = cv2.waitKey(60)
        if key == 27:
            break

            


rospy.init_node("Vision")
main()
rospy.spin()
