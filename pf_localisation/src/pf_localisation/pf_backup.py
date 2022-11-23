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
        self.CLOUD_POINTS = 200
        self.TOP_WEIGHT_PERCENTAGE = 0.95
        
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
        #euclideanDistance = math.sqrt(math.pow(particle.position.x - self.estimatedpose.pose.pose.position.x, 2) + math.pow(particle.position.y - self.estimatedpose.pose.pose.position.y,2))
        euclideanDistance = math.sqrt(math.pow(particle.position.x, 2) + math.pow(particle.position.y,2))
        return euclideanDistance


    def particle_clustering(self, particlePoses):
        """
        Takes the heaviest cluster of particles, and everything that is near it, 
        to recalculate particles and get an estimated pose
        of particles that are within 1 standard deviation rate.
        
        """

        
        euclideanDistances = [] #Initializes a new array to store euclidean distances.

        for pose in particlePoses: #Takes each individual particle in the particle poses array
            euclideanDistances.append(self.euclidean_function(pose)) #sends each pose to the euclidean function to determine each particle's euclidean distance
                                                                     #and adds to array
        
        meanDistance = mean(euclideanDistances) #Calculates the mean of the distances to use as the center of gaussian
        standardDeviation = stdev(euclideanDistances) #Determines the standard deviation from the mean

        if standardDeviation > 3: #If standard deviation its greater than this, don't consider the particle
            remainingParticlesPoses = [] #creates an array for the new particles
            for pose in particlePoses: # for each particle inside the array of particles
                singleEuclideanDistance = self.euclidean_function(pose) #sends the particle to get its euclidian distance
                if meanDistance - standardDeviation < singleEuclideanDistance < meanDistance + standardDeviation: # mean - sigma < mean < mean + sigma #boundary of gaussian 
                    remainingParticlesPoses.append(pose) #appends the particles to the array that fulfill this condition.
            return self.particle_clustering(remainingParticlesPoses) #recursive array with new particles, until it returns the robotEstimatedPose std < 3

        else:
            robotEstimatedPose = Pose() #Creates Robot estimated pose

            for pose in particlePoses: #For each  of the remaining particles in the particle cloud, calculate the mean point
                robotEstimatedPose.position.x = robotEstimatedPose.position.x + pose.position.x # This is the new estimated pose in x sum
                robotEstimatedPose.position.y = robotEstimatedPose.position.y + pose.position.y # New estimated pose in y sum
                robotEstimatedPose.orientation.w = robotEstimatedPose.orientation.w + pose.orientation.w # New estimated orientation w sum
                robotEstimatedPose.orientation.z = robotEstimatedPose.orientation.z + pose.orientation.z # New estimated orientation Z sum

            robotEstimatedPose.position.x = robotEstimatedPose.position.x/len(particlePoses) #divides the sum (x) by the number of particle remaining to get mean x
            robotEstimatedPose.position.y = robotEstimatedPose.position.y/len(particlePoses) #divides the sum (y) by the number of particle remaining to get mean y
            robotEstimatedPose.orientation.w = robotEstimatedPose.orientation.w/len(particlePoses) #divides the sum (w) by the number of particle remaining to get mean w
            robotEstimatedPose.orientation.z = robotEstimatedPose.orientation.z/len(particlePoses) #divides the sum (Z) by the number of particle remaining to get mean z
        
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
        appendedParticles = 0 # To check that the @NUMBER cloudpoints have been added to the array

        while appendedParticles < cloudPoints:
            myPose = Pose() #creates a pose variable, once per cycle to not cause problem when appending
            random_angle = vonmisesvariate(0,0) # generates a random angle between 0 to 2pi
            #random_x = randint(0,width-1)# generates a random position around center of map
            random_x = int(gauss(math.floor(initialpose.pose.pose.position.x/resolution), width/8))
            #random_y = randint(0,height-1) # generates a random position around center of map
            random_y = int(gauss(math.floor(initialpose.pose.pose.position.y/resolution), height/8))
            myPose.position.x = random_x * resolution #Multiplies by the resolution of the map so the correct x position is obtained
            myPose.position.y = random_y * resolution #Multiplies by the resolution of the map so the correct y position is obtained
            myPose.orientation = rotateQuaternion(Quaternion(w=1.0),random_angle) #rotates from default quaternion into new angle

            if random_x < width - 1 and random_y < height - 1:
                if self.occupancy_map.data[random_x + random_y * width] == 0: # Verifies that the particle is created in a white space
                    arrayPoses.poses.append(myPose) #Adds the particle to an array.
                    appendedParticles += 1 #Ready to append the next particle

        # print(appendedParticles)
        return arrayPoses #Returns the array so they are added to the particle cloud

     
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
                remainingWeightPoses.poses.append(myPose) #Adds the particle to an array.
                appendedParticles += 1 #Ready to append the next particle
            



        """ Resampling of topWeight Particles"""
        # ------ Cumulative Distribution initialization
        cumulativeDistributionF = [] #Initializes the cumulative distribution array
        accumulatedWeight = 0 #The initial weight of the particle

        for (particle, weight) in heaviestParticles: #Heaviest particle array has particle data, and weight data, this was already sorted before, created this to make new array
            cumulativeDistributionF.append((particle, accumulatedWeight + weight/weightSum)) # Appends the particle info, and the accumulated weight to create cumulative weight distribution. 
            accumulatedWeight = accumulatedWeight + weight/weightSum #Accumulate weights 
        
        threshold = uniform(0,math.pow(len(heaviestParticles),-1)) #Creates uniform distribution for the threshold to update particles
        cycleNum = 0 # variable for while
        arrayPoses = PoseArray() #creates an array of poses to store poses

        for points in range(0, len(heaviestParticles)): #starts updating threshold and storing positions  of heaviest particles
            while threshold > cumulativeDistributionF[cycleNum][1]:
                cycleNum += 1 
            
            myPose = Pose() #stores the new pose
            myPose.position.x = cumulativeDistributionF[cycleNum][0].position.x + gauss(0,self.ODOM_TRANSLATION_NOISE ) #stores position x
            myPose.position.y = cumulativeDistributionF[cycleNum][0].position.y + gauss(0,self.ODOM_DRIFT_NOISE ) #stores position y
            myPose.orientation = rotateQuaternion(Quaternion(w=1.0), getHeading(cumulativeDistributionF[cycleNum][0].orientation) + 
                            gauss(0, self.ODOM_ROTATION_NOISE)) #stores orientation

            arrayPoses.poses.append(myPose) #appends the pose
            threshold = threshold + math.pow(len(heaviestParticles),-1) #continues updating the threshold


        modifiedPosesArray = arrayPoses #stores the array in the modified array
        modifiedPosesArray.poses = modifiedPosesArray.poses + remainingWeightPoses.poses #combines both array poses


        self.particlecloud = modifiedPosesArray #updates particle cloud


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

        robotEstimatedPose  = self.particle_clustering(self.particlecloud.poses) #calls function that "clusters particles" with means and standard distribution

        return robotEstimatedPose #returns the new estimated pose
