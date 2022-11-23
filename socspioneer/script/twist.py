#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry


def talker():
    # Publishers
    humanMovementPub = rospy.Publisher('robot_1/cmd_vel', Twist, queue_size=100)
    robotRealPosPub = rospy.Publisher('robotReal', Odometry, queue_size=1)
    humanRealPosPub = rospy.Publisher('humanReal', Odometry, queue_size=1)
    
    rospy.init_node('Mover', anonymous=True)

    #Subscribers
    humanMovementGet = rospy.Subscriber('cmd_vel', Twist, human_move, (humanMovementPub))
    robotRealPosGet = rospy.Subscriber('robot_0/base_pose_ground_truth', Odometry, updateRobot, (robotRealPosPub))
    humanRealPosGet = rospy.Subscriber('robot_1/base_pose_ground_truth', Odometry, updateHuman, (humanRealPosPub) )
    
    # rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        #base_data = Twist()
        rospy.spin()
        # rate.sleep()

def human_move(self, movementPub):
    movementPub.publish(self)

def updateRobot(self, cartPub):
    robotOdom = Odometry()
    robotOdom = self
    robotOdom.header.frame_id = "map"
    cartPub.publish(robotOdom)

def updateHuman(self, humanPub):
    humanOdom = Odometry()
    humanOdom = self
    humanOdom.header.frame_id = "map"
    humanPub.publish(humanOdom)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass 
