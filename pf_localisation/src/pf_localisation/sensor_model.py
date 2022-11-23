"""
sensor_model.py
Provides a SensorModel class to calculate particle weights.
"""
import rospy
from . util import getHeading

import math

from . import laser_trace

PI_OVER_TWO = math.pi/2 

class SensorModel(object):
    def __init__(self):
        # ----- Parameters for particle weight calculation
        self.z_hit = 0.95 		# Default probability if we make a hit
        self.z_short = 0.1 	# Probability of a short reading from 
                               # unexpected obstacle (e.g. person or object)
        self.z_max = 0.05 		# Probability of failure to detect an obstacle,
                               # reported as max range
        self.z_rand = 0.05 	# Random noise on all readings
        
        self.sigma_hit = 0.2 		# Noise on hit
        self.lambda_short = 0.1 	# Noise on short reading
        
        # Initialise scan parameters to nothing
        self.set_laser_scan_parameters(0, 0, 0, 0, 0)
        
    def set_laser_scan_parameters(self,num_readings, scan_range_max,
                                  scan_length, scan_angle_min,
                                  scan_angle_max ):
        """
        Set the parameters for laser scanner that this instance is modeling
        
        :Args:
            | num_readings (int): Number of scan readings to be compared with
                                  predictions when computing particle weights
            | scan_range_max (double): Max range scanner can read
            | scan_length (int) : The number of readings in a complete scan
            | scan_angle_min (double): The min. angle of the scanner
            | scan_angle_max (double): The max. angle of the scanner
        """
        # ----- Laser parameters
        self.scan_range_max = scan_range_max
        
        # ----- What points to sample the laser at when calculating particle weight
        reading_step = (scan_length - 1) / (num_readings - 1)
        self.reading_points = [(i, scan_angle_min +
                                ((scan_angle_max - scan_angle_min) *
                                 (float(i) / scan_length)))
                               for i in range(0, scan_length, int(reading_step))]
        
        rospy.loginfo("Sensor model scan parameters set.")
        
    def set_map(self, occupancy_map):
        """
        Set the map that this model should use when calculating expected
        laser readings.
        
        :Args:
            | occupancy_map (sensor_msgs.msg.OccupancyGrid): the map to use
        """
        # ----- Map data
        self.map_width = occupancy_map.info.width
        self.map_height = occupancy_map.info.height
        self.map_resolution = occupancy_map.info.resolution # in m per pixel
        self.map_data =  occupancy_map.data 
        self.map_origin_x = ( occupancy_map.info.origin.position.x +
                             (self.map_width / 2.0) * self.map_resolution )
        self.map_origin_y = ( occupancy_map.info.origin.position.y +
                              (self.map_height / 2.0) * self.map_resolution )
        rospy.loginfo("Sensor model map set.")

    def calc_map_range(self, ox, oy, oa):
        """
        Given a location on the map and a direction, predict the visible
        laser range.
         
        :Args:
            | ox (double): X location of observation
            | oy (double): Y location of observation
            | oa (double): Bearing (from North, in degrees) of the reading
        :Returns:
            | (double) Range (in m) expected to be observed by the laser
        """
        r = laser_trace.map_calc_range(ox, oy, oa, self.map_width,
                                          self.map_height,
                                          self.map_origin_x,
                                          self.map_origin_y,
                                          self.map_resolution,
                                          self.scan_range_max,
                                          self.map_data)
        if r <= self.scan_range_max:
            return r
        else:
            # ----- rospy.logwarn("calc_map_range giving oversized ranges!!")
            return self.scan_range_max
        
    def get_weight(self, scan, pose):
        """
        Compute the likelihood weighting for each of a set of particles 
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): Current observation
            | pose (geometry_msgs.msg.Pose): Particle's estimated location
        :Returns:
            | (double) likelihood weighting for this particle, given the map
              and laser readings
         """
    
        p = 1.0 # Sample weight (not a probability!)
        
        

        
        for i, obs_bearing in self.reading_points:
            # ----- For each range...
            obs_range = scan.ranges[i]
            
            # ----- Laser reports max range as zero, so set it to range_max
            if (obs_range <= 0.0):
                obs_range = self.scan_range_max 
            
            # ----- Compute the range according to the map
            map_range = self.calc_map_range(pose.position.x, pose.position.y,
                                     getHeading(pose.orientation) + obs_bearing)
            pz = self.predict(obs_range, map_range)
            p += pz*pz*pz # Cube probability: reduce low-probability particles 
            
        return p
    
    def predict(self, obs_range, map_range):
        """
        Implementation of AMCL's sensor model.
        
        :Args:
            | obs_range (double): Observed range (i.e. from actual laser)
            | map_range (double): Predicted range (i.e. from map)
        :Returns:
            | (double) Probability [0,1] that we would observe the obs_range
              data given the estimated map location
         """
        pz = 0.0
    
        # ----- Part 1: good, but noisy, hit
        z = obs_range - map_range
        pz += ( self.z_hit *
                math.exp(-(z * z) / (2 * self.sigma_hit * self.sigma_hit)) )
    
        # ----- Part 2: short reading from unexpected obstacle (e.g., a person)
        if z < 0:
            pz += ( self.z_short * self.lambda_short *
                    math.exp(-self.lambda_short*obs_range) )
    
        # ----- Part 3: Failure to detect obstacle, reported as max-range
        if obs_range == self.scan_range_max:
            pz += self.z_max * 1.0
    
        # ----- Part 4: Random measurements
        if obs_range < self.scan_range_max:
            pz += self.z_rand * 1.0/ self.scan_range_max
    
        assert(pz <= 1.0)
        assert(pz >= 0.0)
        
        return pz

   
