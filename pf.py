from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point
from . pf_base import PFLocaliserBase

from . util import rotateQuaternion, getHeading
from sklearn.cluster import DBSCAN
from sklearn.neighbors import KNeighborsRegressor
from scipy.stats import gaussian_kde

import numpy as np
class PFLocaliser(PFLocaliserBase):
       
    def __init__(self, logger, clock):
        # ----- Call the superclass constructor
        super().__init__(logger, clock)
        
        # ----- Set motion model parameters
 
        # ----- Sensor model parameters

        # Logger to track data around implemented methods
        self.logger = logger

        self.ODOM_ROTATION_NOISE = 0.06
        self.ODOM_TRANSLATION_NOISE = 0.325
        self.ODOM_DRIFT_NOISE = 0.325
        self.NUMBER_PREDICTED_READINGS = 150     # Number of readings to predict
        
       
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        poses = []
        # Initial pose
        init_x, init_y, init_th = initialpose.pose.pose.position.x, initialpose.pose.pose.position.y, initialpose.pose.pose.orientation
        
        for i in range(self.NUMBER_PREDICTED_READINGS):
            point = Point()

            # Apply noise parameters to particles
            point.x = init_x + np.random.normal(0, self.ODOM_TRANSLATION_NOISE)
            point.y = init_y + np.random.normal(0, self.ODOM_DRIFT_NOISE)
            point.z = 0.0
            newangle = rotateQuaternion(init_th, np.random.normal(0, self.ODOM_ROTATION_NOISE))

            pose = Pose()
            pose.position = point
            pose.orientation = newangle
            poses.append(pose)

        poseArr = PoseArray()
        poseArr.poses = poses
        return poseArr
        



 
    
    def update_particle_cloud(self, scan):
        """
        Use the supplied laser scan to update the current particle cloud.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update
        """
        
        def systematic_resampling(weights):
            n = len(weights)
            positions = (np.arange(n) + np.random.uniform(0, 1)) / n
            indices = np.zeros(n, dtype=int)

            total_sum = np.cumsum(weights)
            i, j = 0, 0
            while i < n:
                if positions[i] < total_sum[j]:
                    indices[i] = j
                    i += 1
                else:
                    j += 1
            return indices
        
        # Compute weights based on the sensor model
        weights = np.array([self.sensor_model.get_weight(scan, pose) for pose in self.particlecloud.poses])
        weights /= np.sum(weights)

        # Resample indices based on weights
        indices = systematic_resampling(weights)

        # Preallocate new_samples for better performance
        new_samples = [Pose() for _ in range(len(indices))]

        for i, idx in enumerate(indices):
            original_pose = self.particlecloud.poses[idx]
            
            # Apply noise
            new_samples[i].position.x = original_pose.position.x + np.random.normal(0, self.ODOM_TRANSLATION_NOISE)
            new_samples[i].position.y = original_pose.position.y + np.random.normal(0, self.ODOM_DRIFT_NOISE)
            self.logger.info("Cur particle's x: " + str(new_samples[i].position.x))
            self.logger.info("Cur particle's y: " + str(new_samples[i].position.y))
            # Rotate orientation with noise
            noisy_orientation = rotateQuaternion(original_pose.orientation, np.random.normal(0, self.ODOM_ROTATION_NOISE))
            new_samples[i].orientation = noisy_orientation
            self.logger.info("Cur particle's theta: " + str(getHeading(new_samples[i].orientation)))

        self.particlecloud.poses = new_samples
        
        
        


    def estimate_pose(self, k=10):
        """
        Estimate the robot's pose using K-Nearest Neighbors from the particle cloud.

        :param k: Number of neighbors to consider for pose estimation.
        :Return: (geometry_msgs.msg.Pose) Robot's estimated pose.
        """
        data = np.empty((len(self.particlecloud.poses), 6))
        for i in range(len(self.particlecloud.poses)):
            data[i, 0] = self.particlecloud.poses[i].position.x
            data[i, 1] = self.particlecloud.poses[i].position.y
            q = self.particlecloud.poses[i].orientation
            data[i, 2] = q.x
            data[i, 3] = q.y
            data[i, 4] = q.z
            data[i, 5] = q.w
        
        # Split data into positions and orientations
        positions = data[:, :2]
        orientations = data[:, 2:]

        # Create KNN regressor for position and orientation
        knn_position = KNeighborsRegressor(n_neighbors=k)
        knn_orientation = KNeighborsRegressor(n_neighbors=k)

        # Fit the models
        knn_position.fit(positions, positions)
        knn_orientation.fit(positions, orientations)

        # Predict the current position based on the nearest neighbors
        predicted_position = knn_position.predict(positions).mean(axis=0)
        predicted_orientation = knn_orientation.predict(positions).mean(axis=0)

        # Create the new pose
        new_pose = Pose()
        new_pose.position.x = predicted_position[0]
        new_pose.position.y = predicted_position[1]

        new_orientation = Quaternion()
        new_orientation.x = predicted_orientation[0]
        new_orientation.y = predicted_orientation[1]
        new_orientation.z = predicted_orientation[2]
        new_orientation.w = predicted_orientation[3]
        
        new_pose.orientation = new_orientation

        return new_pose
    