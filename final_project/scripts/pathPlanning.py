#!/usr/bin/python3
#This is an implementation of A* algorithm

import rospy
import math
import heapq

import copy
import tf.transformations

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray, Marker

trackingMode = "human"
pathFound = False

#This is the movements the robot takes
class Moving:
    def __init__(self, length, thetaDistance):
        self.length = length
        self.thetaDistance = thetaDistance

#This is the information of the robot, height and width (in X and Y), not including how tall in Z
class Robot:
    def __init__(self, width = 2.0, height = 3.0):
        self.width = width
        self.height = height

class Map:
    def __init__(self, grid):
        self.map = grid # obtains the grid map and saves it in map of map class
        self.width = grid.info.width # obtains the grid info of width and saves it in width of map class
        self.height = grid.info.height # obtains the grid info of height and saves it in height of map class
        self.resolution = grid.info.resolution # obtains the grid resolution

        self.origin = Point() # To obtain and save this as a geometry message of pose. 
        self.origin.x = grid.info.origin.position.x # Stores the origin position of X
        self.origin.y = grid.info.origin.position.y # Stores map origin position of Y

    def coord_by_index(self, i, j):
        if not self.index_in_range(i, j): # Throws error if indexes are out of the map range.
            raise IndexError()
        return self.map.data[i*self.width + j] # Returns the data contained in an index of the map, specific number in list

    def coord_to_indices(self, x, y):
        i = int((y - self.origin.y) / self.resolution) # To adjust the coordinates to be positive numbers in y and divide by the resolution
        j = int((x - self.origin.x) / self.resolution) # To adjust the coordinates to be positive numbers in y and divide by the resolution
        return (i, j) # returns the index of column and row of coordinate

    def index_in_range(self, i, j):
        return 0 <= i < self.height and 0 <= j < self.width # Verifies that the indices are inside the correct range of the map

    def is_allowed(self, state, robot):
        isError = False 
        i, j = self.coord_to_indices(state.x, state.y) # Converts the current coordinates of center of robot, into indices i,j 
        side = int(math.floor((max(robot.width, robot.height) / self.resolution) / 2)) #considering that coordinate is in center, takes into account size of robot

        try:
            for sideI in range(i-side, i+side):
                for sideJ in range(j-side, j+side):
                    cell = self.coord_by_index(sideI, sideJ) # Examines every index of block to know if it contains an object
                    if cell == 100 or cell == -1: # If it contains an object or is out of bounds returns false
                        return False
        except IndexError as e:
            isError = True
        return True and not isError # To return True or False

class State:
    def __init__(self, x=0.0, y=0.0, theta=0.0, parent=None):
        self.x = x # Current State X
        self.y = y # Current State Y
        self.theta = theta # Current angle
        self.parent = parent #previous node

        # Variables for A* algorhytm
        self.g = 0 #g-score --> path cost of the node
        self.h = 0 #heuristic --> depends on distance between node and goal variable, euclidean distance
        self.f = 0 # self.g + self.h  --> f-score  --> Gauging how useful movement is

    @staticmethod
    def robot_pose(pose):
        newState = State() # Initializes the new state of the robot
        newState.x = pose.position.x # Assigns the new position x in the state
        newState.y = pose.position.y # Assigns the new position y to the state
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) # Stores the quaternion for rotation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion) # Using the transformation saves the roll, pitch and yaw
        newState.theta = yaw # The new theta depends on the Yaw
        return newState # returns the new state of the robot

    def eucl_dist(self, theGoal):
        return ((self.x - theGoal.x)**2 + (self.y - theGoal.y)**2)**0.5 # euclidean distance sqrt(x^2 + y^2) 

    def is_same_as(self, theGoal):
        return self.eucl_dist(theGoal) <= 0.1 # think a much better about this constant change distance to speed up algorithm

    def apply(self, Movement):
        newTheta = self.theta + Movement.thetaDistance # Original angle + movement angle
        newX = self.x + math.cos(newTheta) * Movement.length # x + component of x due to angle and movement line
        newY = self.y + math.sin(newTheta) * Movement.length #y + component of y do to angle and movement line
        return State(newX, newY, newTheta) #in class of state records the new positions

    def try_apply(self, _map, Movement, robot):
        modelState = copy.copy(self) # Makes a copy of current state
        modelState.parent = self # Stores the copy of the state as a parent

        goal = self.apply(Movement) # Applies the movement of the variable and stores them in the goal
        goal.parent = self # reference parent to self
        steps_count = max(int(self.eucl_dist(goal) / min(robot.height, robot.width)), 1) # Calculates the number of steps required from robot to goal
        step = 1.0 / steps_count # to find out step size of the robot

        for i in range(steps_count):
            modelState.theta += Movement.thetaDistance * step  # Slightly change angle of robot

            modelState.x += math.cos(modelState.theta) * Movement.length * step # changes the position in x over time slowly
            modelState.y += math.sin(modelState.theta) * Movement.length * step # changes the position in y over time slowly

            if not _map.is_allowed(modelState, robot):
                return None

        return goal

    def to_pose_stamped(self):
        pose = PoseStamped() #This stores the variables we have as a PoseStamped
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.1 #depends on what we put in the robot as Z axis center

        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        return pose

    def to_marker(self, robot):
        marker = Marker() #taking the properties of the robot creates a marker
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0

        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]

        marker.scale.x = robot.width
        marker.scale.y = robot.height
        marker.scale.z = 0.2

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.type = Marker.SPHERE
        marker.action = marker.ADD

        return marker

#Function for planning movement based on heaps https://www.youtube.com/watch?v=58cYFs_W2_s
def replan(map, Movements, robot, start, goal, pub):
    robot_dimension = min(robot.width, robot.height)

    final_state = None
    opened = []
    heapq.heappush(opened, (0.0, start))
    closed = []

    while opened and final_state is None:
        q = heapq.heappop(opened)[1]
        for Movement in Movements:
            successor = q.try_apply(map, Movement, robot)
            pub.publish(q.to_pose_stamped())
            if successor is not None:
                if successor.eucl_dist(goal) < robot_dimension:
                    final_state = successor
                    break
                successor.g = q.g + successor.eucl_dist(q)
                successor.h = successor.eucl_dist(goal)
                successor.f = 0.2 * successor.g + successor.h
                successor.parent = q

                # TODO ->replace<- with new positions
                better_in_opened = any(other_successor.is_same_as(successor) and other_f <= successor.f for other_f, other_successor in opened)
                if not better_in_opened:
                    better_in_closed = any(other_successor.is_same_as(successor) and other_successor.f <= successor.f for other_successor in closed)
                    if not better_in_closed:
                        heapq.heappush(opened, (successor.f, successor))

        closed.append(q)

    return final_state

class TrajectoryPlanner:
    def __init__(self):
        
        self.map = None
        self.start = None
        self.goal = None

        self.Movements = [Moving(0.1, 0),  # forward
                          Moving(-0.1, 0),  # back
                          Moving(0, 1.5708),  # turn left 90
                          Moving(0, -1.5708)] # turn right 90
        
        self.robot = Robot(0.5, 0.5)
        self.is_working = False 

        self.map_subscriber = rospy.Subscriber("map", OccupancyGrid, self.new_map_callback)

        #SHOULD UPDATE BASED ON REQUIREMENTS AND CODE VERSION
        self.start_subscriber = rospy.Subscriber("robotReal", Odometry, self.new_start_callback) #Get's the real position of robot

        if trackingMode == "human":
            self.goal_subscriber = rospy.Subscriber("humanReal", Odometry, self.new_goal_callback) #Path plans for human
            
        self.path_publisher = rospy.Publisher("trajectory", MarkerArray, queue_size=1)
        self.pose_publisher = rospy.Publisher("debug_pose", PoseStamped, queue_size=1)

        self.planner = replan

    def ready_to_plan(self):
        return self.map is not None and self.start is not None and self.goal is not None

    def new_goal_callback(self, humanPose):

        #needs a PoseStamped
        goal_pose = PoseStamped()
        goal_pose.header = humanPose.header
        goal_pose.pose = humanPose.pose.pose

        if not self.is_working:
            self.is_working = True
            new_goal = State.robot_pose(goal_pose.pose)
            if self.map is not None and self.map.is_allowed(new_goal, self.robot):
                self.goal = new_goal
                rospy.loginfo("New goal was set")
                if self.ready_to_plan():
                    self.replan()
            else:
                rospy.logwarn("New goal is bad or no map available")

            self.is_working = False

    def new_start_callback(self, robotPose):
        
        #needs a PoseWithCovarianceStamped
        start_pose = PoseWithCovarianceStamped()
        start_pose.header = robotPose.header
        start_pose.pose = robotPose.pose
        if not self.is_working:
            self.is_working = True
            new_start = State.robot_pose(start_pose.pose.pose)
            if self.map is not None and self.map.is_allowed(new_start, self.robot):
                self.start = new_start
                rospy.loginfo("New start was set")
                if self.ready_to_plan():
                    self.replan()
            else:
                rospy.logwarn("New start is bad or no map available")
            self.is_working = False

    def new_map_callback(self, grid):
        if not self.is_working:
            self.is_working = True
            self.map = Map(grid)
            rospy.loginfo("New map was set")
            if self.ready_to_plan():
                self.replan()
            self.is_working = False

    def replan(self):
        rospy.loginfo("Planning was started")
        final_state = self.planner(self.map, self.Movements, self.robot, self.start, self.goal, self.pose_publisher)

        if final_state is None:
            rospy.loginfo("No path found")
        else:
            # Restore and publish path
            rospy.loginfo("Restoring path from final state...")
            path = self.restore_path(final_state)
            self.path_publisher.publish(path)
            rospy.loginfo("Planning was finished...")
            rospy.loginfo("Path Found...")
            self.reached_goal(True)

    def restore_path(self, final_state):
        current_state = copy.copy(final_state)
        path = MarkerArray()
        pose_id = 0
        while True:
            pose_marker = current_state.to_marker(self.robot)
            pose_marker.id = pose_id
            path.markers.append(pose_marker)

            current_state = current_state.parent
            pose_id += 1

            if current_state is None:
                break
        return path

    def reached_goal(self, goalReached: False):
        return goalReached


def main():
    rospy.init_node("AStar")
    global pathFound
    planner = TrajectoryPlanner()
    rospy.loginfo("is path found" + str(pathFound))
    rospy.spin()
    
    #while not pathFound:
       #rospy.spin()

main()