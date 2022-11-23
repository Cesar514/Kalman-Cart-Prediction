from geometry_msgs.msg import Pose, PoseArray, Quaternion
from . pf_base import PFLocaliserBase
import math
import rospy

from . util import rotateQuaternion, getHeading
from random import Random, gauss, randint, uniform, vonmisesvariate
from statistics import mean, stdev


from time import time


class PFLocaliser(PFLocaliserBase):


    def __init__(self):

        # ----- Assigns an initial number of cloud points
        self.CLOUD_POINTS = 400
        self.TOP_WEIGHT_PERCENTAGE = 0.99
        
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0.015 # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.030 # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.022 # Odometry model y axis (side-to-side) noise
 
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 50 # Number of readings to predict

        # ----- Maybe add something for clusters or kidnapped robot.
        self.MAP_RESOLUTION = self.occupancy_map.info.resolution
        self.WIDTH = self.occupancy_map.info.width
        self.HEIGHT = self.occupancy_map.info.height


    def euclidean_function(self, particle):
        euclideanDistance = math.sqrt(math.pow(particle.position.x - self.estimatedpose.pose.pose.position.x, 2) + math.pow(particle.position.y - self.estimatedpose.pose.pose.position.y,2))
        return euclideanDistance


    def particle_clustering(self, particlePoses):
        
        euclideanDistances = []

        for pose in particlePoses:
            euclideanDistances.append(self.euclidean_function(pose))
        
        meanDistance = mean(euclideanDistances)
        standardDeviation = stdev(euclideanDistances)

        if standardDeviation > 3:
            remainingParticlesPoses = []
            for pose in particlePoses:
                singleEuclideanDistance = self.euclidean_function(pose)
                if meanDistance - standardDeviation < singleEuclideanDistance < meanDistance + standardDeviation:
                    remainingParticlesPoses.append(pose)
            return self.particle_clustering(remainingParticlesPoses)

        else:
            robotEstimatedPose = Pose()

            for pose in particlePoses:
                robotEstimatedPose.position.x = robotEstimatedPose.position.x + pose.position.x
                robotEstimatedPose.position.y = robotEstimatedPose.position.y + pose.position.y
                robotEstimatedPose.orientation.w = robotEstimatedPose.orientation.w + pose.orientation.w
                robotEstimatedPose.orientation.z = robotEstimatedPose.orientation.z + pose.orientation.z

            robotEstimatedPose.position.x = robotEstimatedPose.position.x/len(particlePoses)
            robotEstimatedPose.position.y = robotEstimatedPose.position.y/len(particlePoses)
            robotEstimatedPose.orientation.w = robotEstimatedPose.orientation.w/len(particlePoses)
            robotEstimatedPose.orientation.z = robotEstimatedPose.orientation.z/len(particlePoses)
        
            return robotEstimatedPose


    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise. Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot is also set here.     
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        
        """ Things to initialize """
        arrayPoses = PoseArray() # The Array we are going to return for the cloud
        width = self.occupancy_map.info.width # The width of the map, so particles only span inside of the width
        height = self.occupancy_map.info.height # The height of the map so particle only span inside the heigh of the map
        resolution = self.occupancy_map.info.resolution #gives the resolution of the map
        cloudPoints = self.CLOUD_POINTS  # Is the number of cloud points we are calculating
        appendedParticles = 0 # To check that the 250 cloudpoints have been added to the array

        while appendedParticles < cloudPoints:
            myPose = Pose() #creates a pose variable, once per cycle to not cause problem when appending
            random_angle = vonmisesvariate(0,0) # generates a random angle between 0 to 2pi
            random_x = randint(0,width-1)# generates a random position around center of map
            #randox_x = gauss(initialpose.x, width/8)
            random_y = randint(0,height-1) # generates a random position around center of map
            #randox_y = gauss(initialpose.y, width/8)
            myPose.position.x = random_x * resolution #Multiplies by the resolution of the map so the correct x position is obtained
            myPose.position.y = random_y * resolution #Multiplies by the resolution of the map so the correct y position is obtained
            myPose.orientation = rotateQuaternion(Quaternion(w=1.0),random_angle) #rotates from default quaternion into new angle

            if self.occupancy_map.data[random_x + random_y * width] == 0: # Verifies that the particle is created in a white space
                arrayPoses.poses.append(myPose)
                appendedParticles += 1

        # print(appendedParticles)
        return arrayPoses

     
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """

        "Initialize Variables"
        cloudPoints = self.CLOUD_POINTS  # Is the # of cloud points we have
        weights = [] # Array for storing the weights of each particle
        

        "Scan the weights of each particle"
        for eachParticle in self.particlecloud.poses:
            myPose = Pose() # A pose for each of the particles in the particle cloud
            myPose = eachParticle #assigns that particle to the pose
            weights.append((eachParticle, self.sensor_model.get_weight(scan, eachParticle))) # Creates a tuple with the particle position and weights
        
        sortedWeights = sorted(weights, key=lambda higherWeights: higherWeights[1], reverse=True) # Puts higher weight particles at top of array to guarantee copies
        heaviestParticles = sortedWeights[0:int(self.CLOUD_POINTS * self.TOP_WEIGHT_PERCENTAGE)] #Takes the % heaviest particles in a new array

        weightSum = sum(higherWeights[1] for higherWeights in heaviestParticles) #Does the sum of weights for top particles


        " Particles to add via new random position generation "
        remainingWeightPoses = PoseArray() #The Array we are going to return for the cloud
        width = self.occupancy_map.info.width #The width of the map, so particles only span inside of the width
        height = self.occupancy_map.info.height #The height of the map so particle only span inside the heigh of the map
        resolution = self.occupancy_map.info.resolution #gives the resolution of the map
        remainingCloudPoints = cloudPoints * (1-self.TOP_WEIGHT_PERCENTAGE) # Is the number of cloud points we are now randomly determining
        appendedParticles = 0 # To check that the remaining cloudpoints have been added

        while appendedParticles < remainingCloudPoints:
            myPose = Pose() #creates a pose variable, once per cycle to not cause problem when appending
            random_angle = vonmisesvariate(0,0) # generates a random angle between 0 to 2pi
            random_x = randint(0,width-1)# generates a random position around center of map 
            random_y = randint(0,height-1) # generates a random position around center of map
            myPose.position.x = random_x * resolution #Multiplies by the resolution of the map so the correct x position is obtained
            myPose.position.y = random_y * resolution #Multiplies by the resolution of the map so the correct y position is obtained
            myPose.orientation = rotateQuaternion(Quaternion(w=1.0),random_angle) #rotates from default quaternion into new angle

            if self.occupancy_map.data[random_x + random_y * width] == 0: # Verifies that the particle is created in a white space
                remainingWeightPoses.poses.append(myPose)
                appendedParticles += 1
            



        """ Resampling of topWeight Particles"""
        # ------ Cumulative Distribution initialization
        cumulativeDistributionF = []
        accumulatedWeight = 0

        for (particle, weight) in heaviestParticles:
            cumulativeDistributionF.append((particle, accumulatedWeight + weight/weightSum))
            accumulatedWeight = accumulatedWeight + weight/weightSum
        
        threshold = uniform(0,math.pow(len(heaviestParticles),-1))
        cycleNum = 0 # variable for while
        arrayPoses = PoseArray()

        for points in range(0, len(heaviestParticles)):
            while threshold > cumulativeDistributionF[cycleNum][1]:
                cycleNum += 1
            
            myPose = Pose()
            myPose.position.x = cumulativeDistributionF[cycleNum][0].position.x + gauss(0,self.ODOM_TRANSLATION_NOISE )
            myPose.position.y = cumulativeDistributionF[cycleNum][0].position.y + gauss(0,self.ODOM_DRIFT_NOISE )
            myPose.orientation = rotateQuaternion(Quaternion(w=1.0), getHeading(cumulativeDistributionF[cycleNum][0].orientation) + 
                            gauss(0, self.ODOM_ROTATION_NOISE))

            arrayPoses.poses.append(myPose)
            threshold = threshold + math.pow(len(heaviestParticles),-1)


        modifiedPosesArray = arrayPoses
        modifiedPosesArray.poses = modifiedPosesArray.poses + remainingWeightPoses.poses #combines both array poses


        self.particlecloud = modifiedPosesArray


    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """

        robotEstimatedPose  = self.particle_clustering(self.particlecloud.poses)

        return robotEstimatedPose
