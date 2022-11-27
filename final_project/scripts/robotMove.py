#!/usr/bin/python3
#This makes the robot move

import rospy
import math

import copy
import tf.transformations

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32MultiArray


currentPose = Pose()
goalReached = True
goalX = None
goalY = None
goalTheta = None
cleanTheta = None
currentNumberPoses = None
previousNumberPoses = None


class TrajectoryTaking:
    def __init__(self):

        self.path = Path()
        self.movementList = []
        self.multiplierRate = 2

        self.roboticSpeed = Twist()
        self.roboticSpeed.linear.x = 0
        self.roboticSpeed.angular.z = 0 #(1.5708/2)
        
        robotMovementPub = rospy.Publisher("/robot_0/cmd_vel", Twist, queue_size=1)

        #self.path_subscriber = rospy.Subscriber("ordered_path", Path, self.save_path)
        self.path_subscriber = rospy.Subscriber("list_movements", Float32MultiArray, self.save_movements, queue_size=1)
        self.position_saving = rospy.Subscriber("robotReal", Odometry, self.save_odometryValues, queue_size=1, buff_size=2**24) # Get's the real position of robot
        self.position_subscriber = rospy.Subscriber("robotReal", Odometry, self.robot_movements, (robotMovementPub), queue_size=1, buff_size=2**24) #Published Data

        self.rate = rospy.Rate(60) # 60 Hz
        #self.rate.sleep() # Sleeps for 1/HZ rate sec

    

    def save_path(self, newPath):
        self.path = newPath
        rospy.loginfo("Path saved")

    
    def save_movements(self, newMovements):
        self.movementList = []
        if len(newMovements.data) > 0:
            i = 0
            while i < len(newMovements.data):
                addMovement = (round(newMovements.data[i],4) , newMovements.data[i+1])
                self.movementList.append(addMovement)
                i = i + 2
        rospy.loginfo("Movements saved")

    def save_odometryValues(self, position):
        global currentPose
        currentPose = position.pose

    def verifications(self, pose):
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) # Stores the quaternion for rotation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion) # Using the transformation saves the roll, pitch and yaw
        return yaw

    def looping(self, currentNumberPoses, x, y, theta, translation, rotation, pub):
        return

    def robot_movements(self, humanMove, pub):
        global currentPose, goalReached, goalX, goalY, goalTheta, cleanTheta, currentNumberPoses
        #rospy.loginfo("movements" + str((self.movementList)))
        #rospy.loginfo("path" + str(len(self.path.poses)))

        #if len(self.path.poses) > 0:
            #rospy.loginfo(len(self.path.poses))
            #currentNumberPoses = len(self.path.poses)
        
        rospy.loginfo(currentPose.pose.position)
        if len(self.movementList) > 1:
        
            speed = 2.0
            savedPose = currentPose.pose
            savedX = savedPose.position.x
            savedY = savedPose.position.y
            savedYaw = self.verifications(savedPose)

            if self.movementList[1][1] > 0.1: #Rotation 
                speed = 0.4
                self.roboticSpeed.linear.x = self.movementList[1][0]*speed
                self.roboticSpeed.angular.z = self.movementList[1][1]*speed

            elif self.movementList[1][1] < -0.1: #Rotation
                speed = 0.4
                self.roboticSpeed.linear.x = self.movementList[1][0]*speed
                self.roboticSpeed.angular.z = self.movementList[1][1]*speed

            elif self.movementList[1][0] > 0.05: #Translation
                speed = 2.5
                self.roboticSpeed.linear.x = self.movementList[1][0]*speed
                self.roboticSpeed.angular.z = self.movementList[1][1]*speed

            else: # Translation
                speed= 2.5
                self.roboticSpeed.linear.x = self.movementList[1][0]*speed
                self.roboticSpeed.angular.z = self.movementList[1][1]*speed

            pub.publish(self.roboticSpeed)

            if goalReached:
                currentNumberPoses = len(self.movementList)
                goalX = savedX + self.movementList[1][0]*math.cos(savedYaw)
                goalY = savedY + self.movementList[1][0]*math.sin(savedYaw)

                goalTheta = savedYaw + self.movementList[1][1]
                if goalTheta < -math.pi:
                    goalTheta = ((goalTheta%math.pi))
                elif goalTheta > math.pi:
                    goalTheta = -((goalTheta %math.pi))
                else:
                    goalTheta = goalTheta
                    
                #rospy.loginfo("current Position X " + str(currentPose.pose))
                rospy.loginfo("goalY" + str(goalY))
                rospy.loginfo("goalX" + str(goalX))
                rospy.loginfo("goalTheta" + str(goalTheta))

            currentPosition = currentPose.pose
            currentYaw = self.verifications(currentPosition)
        

            if not((goalX - 0.01 < currentPosition.position.x <= goalX + 0.01) and (goalY - 0.01 < currentPosition.position.y <= goalY + 0.01) and (goalTheta - 0.01 < (currentYaw) <= goalTheta + 0.01)):#and(())and(goalTheta - 0.01 < currentYaw <= goalTheta + 0.01))"

                pub.publish(self.roboticSpeed)
                #rospy.Time.now().to_sec()
                self.rate.sleep()
                currentPosition = currentPose.pose
                currentYaw = self.verifications(currentPosition) 
                rospy.loginfo("currentYaw" + str(currentYaw))
                rospy.loginfo("goalY" + str(goalY))
                rospy.loginfo("goalX" + str(goalX))
                rospy.loginfo("goalTheta" + str(goalTheta))
                goalReached = False
            
                if currentNumberPoses != len(self.movementList):
                    timeZero = rospy.Time.now().to_sec()     
                    timeOne = rospy.Time.now().to_sec()
                    goalReached = True
                    while((timeOne - timeZero) < 1.2):
                        timeOne = rospy.Time.now().to_sec()
                        self.roboticSpeed.linear.x = 0
                        self.roboticSpeed.angular.z = 0
                        rospy.loginfo("Recalculating.....")
                        pub.publish(self.roboticSpeed)
                    return
            else:
                self.roboticSpeed.linear.x = 0
                self.roboticSpeed.angular.z = 0
                pub.publish(self.roboticSpeed)
                goalReached = True
                rospy.Rate(10)
                self.rate.sleep()
                #self.path.poses.pop(0)
                self.movementList.pop(0)


            #self.looping(currentNumberPoses, savedX, savedY, savedYaw, self.movementList[0][0], self.movementList[0][1], pub)
            
            
            
            


    

        """
        while((timeOne - timeZero) < 1/speed):
                pub.publish(self.roboticSpeed)
                timeOne = rospy.Time.now().to_sec()
                start_pose.pose = humanMove.pose
                
                if currentNumberPoses != len(self.movementList):
                    
                    break

                #if currentNumberPoses != len(self.path.poses):
                    #break
        return
        """



def main():
    rospy.init_node("RobotControl")
    mover = TrajectoryTaking()
    rospy.spin()
main()