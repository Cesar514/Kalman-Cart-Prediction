#!/usr/bin/python3
#This makes the robot move

import rospy
import math

import copy
import tf.transformations

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32MultiArray

class Moving:
    def __init__(self, length, thetaDistance):
        self.length = length
        self.thetaDistance = thetaDistance

class TrajectoryTaking:
    def __init__(self):

        self.Movements = [Moving(0.1, 0),  # forward
                          #Moving(-0.1, 0),  # back           
                          Moving(0, 1.5708), # turn left 90
                          Moving(0, -1.5708), # turn right 90
                          ] 
        self.path = Path()
        self.movementList = []
        self.multiplierRate = 2

        self.roboticSpeed = Twist()
        self.roboticSpeed.linear.x = 0
        self.roboticSpeed.angular.z = 0 #(1.5708/2)
        
        robotMovementPub = rospy.Publisher("/robot_0/cmd_vel", Twist, queue_size=100)

        self.path_subscriber = rospy.Subscriber("ordered_path", Path, self.save_path)
        self.path_subscriber = rospy.Subscriber("list_movements", Float32MultiArray, self.save_movements)
        self.position_subscriber = rospy.Subscriber("robotReal", Odometry, self.robot_movements, (robotMovementPub)) #Get's the real position of robot

        self.rate = rospy.Rate(60) # 2 Hz
        #self.rate.sleep() # Sleeps for 1/HZ rate sec

    def save_path(self, newPath):
        self.path = newPath
        rospy.loginfo_once("Path saved")

    
    def save_movements(self, newMovements):
        if len(newMovements.data) > 0:
            i = 0
            while i < len(newMovements.data):
                addMovement = (round(newMovements.data[i] * self.multiplierRate,4) , newMovements.data[i+1] * self.multiplierRate)
                self.movementList.append(addMovement)
                i = i + 2
        rospy.loginfo_once(self.movementList)
        rospy.loginfo_once("Movements saved")

    def robot_movements(self, humanMove, pub):
        #rospy.loginfo(str(self.path))

        if len(self.path.poses) > 0:
            #rospy.loginfo(len(self.path.poses))

            self.roboticSpeed.linear.x = self.movementList[0][0]
            self.roboticSpeed.angular.z = self.movementList[0][1]
            #rospy.loginfo_once(str(self.path.poses))
            #rospy.loginfo(str(self.roboticSpeed))

            timeZero = rospy.Time.now().to_sec()
            
            timeOne = rospy.Time.now().to_sec()
            speed = 2.0

            #rospy.loginfo(self.movementList[0][0])
            #rospy.loginfo(self.movementList[0][1])

            if self.movementList[0][1] > 0.1:
                speed = 1.0
            elif self.movementList[0][1] < -0.1:
                speed = 1.0
            else:
                speed= 2.0
            #pub.publish(self.roboticSpeed)
            while((timeOne - timeZero) < 1/speed):
                pub.publish(self.roboticSpeed)
                timeOne = rospy.Time.now().to_sec()
            
            
            self.roboticSpeed.linear.x = 0
            self.roboticSpeed.angular.z = 0
            pub.publish(self.roboticSpeed)

            self.rate = rospy.Rate(20) # Hz
            self.rate.sleep()
            self.path.poses.pop(0)
            self.movementList.pop(0)
            
            
            return
        return

    def verifications():
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) # Stores the quaternion for rotation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion) # Using the transformation saves the roll, pitch and yaw


def main():
    rospy.init_node("RobotControl")
    mover = TrajectoryTaking()
    rospy.spin()
main()