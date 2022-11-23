#!/usr/bin/python3
# The Free Spirit Cesar514
import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

averageDistance = 0.0

def callback(data):
    global averageDistance
    averageDistance = 0.0
    for x in range(245,257):
        averageDistance = data.ranges[x] + averageDistance
    averageDistance = averageDistance/12

def solver():
    global averageDistance
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    rospy.Subscriber('base_scan',LaserScan, callback)
    rospy.init_node('ScanTwist', anonymous=True)
    #rate = rospy.Rate(60) # 10hz

    while not rospy.is_shutdown():
        base_data = Twist()
        turningTime = 0
        movingTime = 0
        rospy.Subscriber('base_scan',LaserScan, callback)

        base_data.angular.z = 0
        base_data.linear.x = 0

        while averageDistance > 1:
            print(averageDistance)
            pub.publish(straight(base_data))
            rospy.Subscriber('base_scan',LaserScan, callback)
            time.sleep(0.1)

        while turningTime < 8:
            pub.publish(turn_left(base_data))
            time.sleep(0.240)
            turningTime = turningTime + 1

        base_data.angular.z = 0
        base_data.linear.x = 0


def turn_left(base_data):
    base_data.linear.x = 0
    base_data.angular.z = -1
    return base_data

def turn_right(base_data):
    base_data.angular.z = 1
    return base_data

def straight(base_data):
    base_data.linear.x = 0.5
    return base_data


if __name__ == '__main__':
    try:
        solver()
    except rospy.ROSInterruptException:
        pass 
