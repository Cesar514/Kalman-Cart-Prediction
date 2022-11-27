#!/usr/bin/python3
#This is an implementation of A* algorithm

#Imports of ros, math, heap library, copy, and transformations for quaternions
import rospy, math, heapq, copy, tf.transformations

#Import for map, current position, and to display path
from nav_msgs.msg import OccupancyGrid, Odometry, Path

"""These are needed for the points and poses"""
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, PoseWithCovarianceStamped

"""To visualize a real size marker"""
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Float32MultiArray

calcMode = "manhattan"
#calcMode = "euclidean"
#calcMode = "average"

#trackingMode = "battery"
trackingMode = "human"
#trackingMode = "paying"
#trackingMode = "selection"

savedMode = trackingMode
#savedModeTwo =  trackingMode


myClickedPoint = Point()
distance = 0.7 #Distance from objective
varDist = 0.1 #Distance from exact position each step

savedGoal = None
savedEuclidean = 0.0
enablePlanning = True

batteryTotal = 100.00
batteryTimeIn = 0
batteryTimeOut = 0 


class Moving:
    """This is for the movements the robot takes, line"""
    def __init__(self, length=0, thetaDistance=0):
        self.length = length
        self.thetaDistance = thetaDistance

class Robot:
    """"This is the information of the robot, height and width (in X and Y), not including how tall in Z"""
    def __init__(self, width = 1.0, height = 1.0):
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

        self.choosenMovement = Moving()

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
        if calcMode == "euclidean":
            return ((self.x - theGoal.x)**2 + (self.y - theGoal.y)**2)**0.5 # euclidean distance sqrt(x^2 + y^2)
        if calcMode == "manhattan":
            return abs(self.x - theGoal.x) + abs(self.y - theGoal.y)  # euclidean distance abs(x-x) + abs(y-y)
        if calcMode == "average":
            return (abs(self.x - theGoal.x) + abs(self.y - theGoal.y) + ((self.x - theGoal.x)**2 + (self.y - theGoal.y)**2)**0.5)/2  #euclidean+manhattan/2

    def is_same_as(self, theGoal, variable = 0.1):
        global varDist
        return self.eucl_dist(theGoal) <= varDist# think a much better about this constant change distance to speed up algorithm

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
        marker.scale.z = 0.25

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.type = Marker.SPHERE
        marker.action = marker.ADD

        return marker

#Function for planning movement based on heaps https://www.youtube.com/watch?v=58cYFs_W2_s
def replan(map, Movements, robot, start, goal):
    robot_dimension = min(robot.width, robot.height)

    final_state = None
    opened = []
    heapq.heappush(opened, (0.0, start))
    closed = []

    while opened and final_state is None:
        q = heapq.heappop(opened)[1]
        for Movement in Movements:
            successor = q.try_apply(map, Movement, robot)

            if successor is not None:
                if successor.eucl_dist(goal) < robot_dimension + distance: #Distance from goals
                    final_state = successor
                    break
                successor.g = q.g + successor.eucl_dist(q)
                successor.h = successor.eucl_dist(goal)
                successor.f = 0.2 * successor.g + successor.h
                successor.parent = q
                successor.choosenMovement = Movement

                better_in_opened = any(other_successor.is_same_as(successor) and other_f <= successor.f for other_f, other_successor in opened)
                if not better_in_opened:
                    better_in_closed = any(other_successor.is_same_as(successor) and other_successor.f <= successor.f for other_successor in closed)
                    if not better_in_closed:
                        heapq.heappush(opened, (successor.f, successor))

        closed.append(q)

    return final_state

class robot_moving():
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

class TrajectoryPlanner:
    def __init__(self):
        
        self.map = None
        self.start = None
        self.goal = None
        self.pathFound = False
            # forward #backward #right90 # left90
        self.Movements = [Moving(0.1, 0), Moving(-0.1, 0), Moving(0, -1.5708), Moving(0, 1.5708)] 
        self.robot = Robot(0.85, 0.85)
        self.is_working = False
        self.currentRobotPosition = Pose()

        # Subscribes to the map
        self.map_subscriber = rospy.Subscriber("map", OccupancyGrid, self.new_map_callback)

        # Subscribes to REAL robot position
        self.start_subscriber = rospy.Subscriber("robotReal", Odometry, self.new_start_callback) #Get's the real position of robot
        
        # Subscribes to REAL human position
        self.goal_subscriber = rospy.Subscriber("humanReal", Odometry, self.new_goal_callback) #Path plans for human

        # Subscribes to clicked point
        self.map_subscriber = rospy.Subscriber("/clicked_point", PointStamped, self.saved_point)

        # Publishes trajectory in MarkerArray Mode
        self.path_publisher = rospy.Publisher("trajectory", MarkerArray, queue_size=30)
        
        # Publishes the trajectory in correct order
        self.real_publisher = rospy.Publisher("ordered_path", Path, queue_size=30)

        # Sends the message with the positions of the trajectory
        self.message_publisher = rospy.Publisher("list_movements", Float32MultiArray, queue_size=30)

        self.planner = replan

    def ready_to_plan(self):
        global savedEuclidean
        if self.map is not None and self.start is not None and self.goal is not None:
            if savedGoal != None:
                savedEuclidean = self.euclidean_distance(self.goal.x, self.goal.y, savedGoal.x, savedGoal.y)
            #rospy.loginfo(savedEuclidean)
        return self.map is not None and self.start is not None and self.goal is not None

    def euclidean_distance(self, xBat, yBat, xCurr, yCurr):
        if calcMode == "euclidean":
            return ((xCurr - xBat)**2 + (yCurr - yBat)**2)**0.5 # euclidean distance sqrt(x^2 + y^2)
        if calcMode == "manhattan":
             return (abs(xCurr - xBat) + abs(yCurr - yBat))
        if calcMode == "average":
            return ((abs(xCurr - xBat) + abs(yCurr - yBat)) + ((xCurr - xBat)**2 + (yCurr - yBat)**2)**0.5)/2  #euclidean+manhattan/2

    def saved_point(self, point):
        global myClickedPoint
        myPoint = Point()
        myPoint.x = point.point.x
        myPoint.y = point.point.x

        myClickedPoint = myPoint

    def new_goal_callback(self, myGoal):
        global distance, varDist, batteryTotal, batteryTimeIn, batteryTimeOut, enablePlanning

        batteryTimeOut = rospy.Time.now().to_sec() 
        if(batteryTimeOut - batteryTimeIn > 1):    
            batteryTotal = batteryTotal - 1
            rospy.loginfo("CurrentBattery: ", batteryTotal)
            batteryTimeIn = rospy.Time.now().to_sec()
            if batteryTotal <= 30:
                trackingMode = "battery"
                #enablePlanning = True
            else:
                trackingMode = savedMode

        if trackingMode == "human":
            goal_pose = PoseStamped()
            goal_pose.header = myGoal.header
            goal_pose.pose = myGoal.pose.pose
            distance = 0.15
            varDist = 0.1
        
        if trackingMode == "paying":
            goal_pose = PoseStamped()
            goal_pose.header = myGoal.header
            payingPose = Pose()
            payingPose.position.x = -7
            payingPose.position.y = -17.5
            goal_pose.pose = payingPose
            distance = 0.1
            varDist = 0.1

        if trackingMode == "battery":
            goal_pose = PoseStamped()
            goal_pose.header = myGoal.header
            closestBattery = Pose()
            distance = 0.1
            varDist = 0.0999

            """
            Battery1 (-2 -14.5)
            Battery2 (-16 -3.25)
            Battery3 (-15 15.5)
            Battery4 (2.75 15.5)
            Battery5 (16 15.5)
            Battery6 (18.25 -3.25)
            Battery7 (12.5 -14.5)
            """
            #Saves battery 1 position
            batteryEuclidean = self.euclidean_distance(-2,-14.5,self.currentRobotPosition.position.x, self.currentRobotPosition.position.y)
            closestBattery.position.x = -2
            closestBattery.position.y = -14.5
            
            #Saves battery 2 position
            if(self.euclidean_distance(-16,-3.25,self.currentRobotPosition.position.x, self.currentRobotPosition.position.y) < batteryEuclidean):
                batteryEuclidean = self.euclidean_distance(-16,-3.25,self.currentRobotPosition.position.x, self.currentRobotPosition.position.y)
                closestBattery.position.x = -16
                closestBattery.position.y = -3.25

            #Saves battery 3 position
            if(self.euclidean_distance(-15, 15.5,self.currentRobotPosition.position.x, self.currentRobotPosition.position.y) < batteryEuclidean):
                batteryEuclidean = self.euclidean_distance(-15, 15.5,self.currentRobotPosition.position.x, self.currentRobotPosition.position.y)
                closestBattery.position.x = -15 
                closestBattery.position.y = 15.5

            #Saves battery 4 position
            if(self.euclidean_distance(2.75, 15.5,self.currentRobotPosition.position.x, self.currentRobotPosition.position.y) < batteryEuclidean):
                batteryEuclidean = self.euclidean_distance(2.75, 15.5,self.currentRobotPosition.position.x, self.currentRobotPosition.position.y)
                closestBattery.position.x = 2.75 
                closestBattery.position.y = 15.5
            
            #Saves battery 5 position
            if(self.euclidean_distance(16, 15.5,self.currentRobotPosition.position.x, self.currentRobotPosition.position.y) < batteryEuclidean):
                batteryEuclidean = self.euclidean_distance(16, 15.5,self.currentRobotPosition.position.x, self.currentRobotPosition.position.y)
                closestBattery.position.x = 16 
                closestBattery.position.y = 15.5

            #Saves battery 6 position
            if(self.euclidean_distance(18.25, -3.25,self.currentRobotPosition.position.x, self.currentRobotPosition.position.y) < batteryEuclidean):
                batteryEuclidean = self.euclidean_distance(18.25, -3.25,self.currentRobotPosition.position.x, self.currentRobotPosition.position.y)
                closestBattery.position.x = 18.25 
                closestBattery.position.y = -3.25

            #Saves battery 7 position
            if(self.euclidean_distance(12.5, -14.5,self.currentRobotPosition.position.x, self.currentRobotPosition.position.y) < batteryEuclidean):
                batteryEuclidean = self.euclidean_distance(12.5, -14.5,self.currentRobotPosition.position.x, self.currentRobotPosition.position.y)
                closestBattery.position.x = 12.5 
                closestBattery.position.y = -14.5
            
            if batteryEuclidean <= 1.1:
                trackingMode = savedMode
            
            goal_pose.pose = closestBattery

            #trackingMode = "none"


        if trackingMode == "selection":
            self.rate = rospy.Rate(0.3) # Hz
            self.rate.sleep()

            goal_pose = PoseStamped()
            goal_pose.header = myGoal.header
            selectedPose = Pose()
            selectedPose.position.x = myClickedPoint.x
            selectedPose.position.y = myClickedPoint.y
            goal_pose.pose = selectedPose
            distance = 0.1
            varDist = 0.1

            self.rate = rospy.Rate(60) # Hz


        if not self.is_working:
            self.is_working = True
            new_goal = State.robot_pose(goal_pose.pose)

            if self.map is not None and self.map.is_allowed(new_goal, self.robot):
                self.goal = new_goal
                #rospy.loginfo("Goal")

                if self.ready_to_plan():
                    self.replan_message()

            else:
                rospy.logwarn("No goal")
                msg = Float32MultiArray()
                msg.data = []
                self.message_publisher.publish(msg)

        self.is_working = False
    
    def new_start_callback(self, robotPose):
        #needs a PoseWithCovarianceStamped
        start_pose = PoseWithCovarianceStamped()
        start_pose.header = robotPose.header
        start_pose.pose = robotPose.pose

        self.currentRobotPosition = start_pose.pose.pose

        if not self.is_working:
            self.is_working = True
            new_start = State.robot_pose(start_pose.pose.pose)
            if self.map is not None and self.map.is_allowed(new_start, self.robot):
                self.start = new_start
                #rospy.loginfo("Position")

                if self.ready_to_plan():
                    self.replan_message()
            else:
                rospy.logwarn("No position")
                msg = Float32MultiArray()
                msg.data = []
                self.message_publisher.publish(msg)
            self.is_working = False



    def new_map_callback(self, grid):
        if not self.is_working:
            self.is_working = True
            self.map = Map(grid)
            rospy.loginfo("Map set")
            if self.ready_to_plan():
                self.replan_message()
            self.is_working = False


    def replan_message(self):
        global enablePlanning
        global savedGoal


        if savedEuclidean > 0.15 or enablePlanning:
            msg = Float32MultiArray()
            msg.data = []
            self.message_publisher.publish(msg)
            rospy.loginfo("Planning started")
            final_state = self.planner(self.map, self.Movements, self.robot, self.start, self.goal)
            rospy.loginfo(savedEuclidean)
            
            savedGoal = self.goal
            self.goal = None
            self.start = None
            if final_state is None:
                rospy.loginfo("No path")
            else:
                # Restore and publish path
                rospy.loginfo("Uploading Path")
                path, theMovements, pathMarker = self.restore_path(final_state)
                self.path_publisher.publish(pathMarker)
                msg.data = theMovements
                self.message_publisher.publish(msg)
                self.real_publisher.publish(path)

                rospy.loginfo("Planning completed")
                
                enablePlanning = False
                #rospy.loginfo(str(enablePlanning))


    def restore_path(self, final_state):
        current_state = copy.copy(final_state)
        path = Path()
        pathMarker = MarkerArray()
        theMovements = []
        pose_id = 0

        while True:
            pose_marker = current_state.to_marker(self.robot)
            pose_marker.id = pose_id
            pathMarker.markers.insert(0, pose_marker)
            #pathMarker.markers.header = pose_id

            pose = PoseStamped()
            choosenMovement = copy.copy(current_state.choosenMovement)
            pose = current_state.to_pose_stamped()
            path.header.stamp = rospy.Time.now()
            path.header.frame_id = "map"
            path.poses.insert(0, pose)

            #movementTuple = (float(choosenMovement.length), float(choosenMovement.thetaDistance))
            theMovements.insert(0,float(choosenMovement.thetaDistance))
            theMovements.insert(0,float(choosenMovement.length))
            

            current_state = current_state.parent
            pose_id += 1

            if current_state is None:
                break
        return path, theMovements, pathMarker

    def reached_goal(self, goalReached: False):
        return goalReached





def main():
    rospy.init_node("AStar")
    planner = TrajectoryPlanner()
    rospy.spin()
main()