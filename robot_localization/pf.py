#!/usr/bin/env python3

""" This is the starter code for the robot localization project """

import rclpy
from threading import Thread
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import ParticleCloud, Particle
from nav2_msgs.msg import Particle as Nav2Particle
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from rclpy.duration import Duration
import random
import math
from math import sin, cos, pi
from scipy.stats import norm
import time
import numpy as np
from occupancy_field import OccupancyField
from helper_functions import TFHelper, draw_random_sample
from rclpy.qos import qos_profile_sensor_data
from angle_helpers import quaternion_from_euler


class Particle(object):
    """ Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self, parent_particle=None, pose_and_ranges={'coords': [(0.0, 0.0), .5], 'theta': [0.0, pi/2]}):
        """ Construct a new Particle
            parent_particle: Particle object used to generate reseeded Particle object
            pose_and_ranges: Dictionary with string keys and list values. Item 0 in list is the value and 
                item 2 is the soft maximum range (3 standard deviations gaussian) of the inputted value

                pose_and_ranges values~
                coords: [(x coord, y coord) floats of cartesian seed position, soft max range after randomizing]
                theta: [float of angle in radians, float soft max angle range after randomizing]
        """
        if parent_particle:
            pose_and_ranges = parent_particle.poses_and_ranges

        self.poses_and_ranges = pose_and_ranges
        self.w = 1.0

        # use 1/3 of soft max ranges to get standard deviation for generation
        self.theta = random.gauss(
            self.poses_and_ranges['theta'][0], self.poses_and_ranges['theta'][1]/3)
        self.x = random.gauss(
            self.poses_and_ranges['coords'][0][0], self.poses_and_ranges['coords'][1]/3)
        self.y = random.gauss(
            self.poses_and_ranges['coords'][0][1], self.poses_and_ranges['coords'][1]/3)

    def get_transform(self):
        """
        return the transformation matrix from the particle frame to the map frame

        Args:
            None
        Returns:
            T_particle_to_map: a 3 x 3 numpy array representing the transformation matrix from the particle's frame to map
        """
        return np.array([[cos(self.theta), -sin(self.theta), self.x],
                         [sin(self.theta), cos(self.theta), self.y], [0, 0, 1]])

    def as_pose(self):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        q = quaternion_from_euler(0, 0, self.theta)
        return Pose(position=Point(x=self.x, y=self.y, z=0.0),
                    orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

    def update_pose(self, delta_pos_particle, delta_ang):
        """
        Using the change in the particle's position, expressed in its own reference frame, 
        and the change in the particle's angle, 
        update the position of the particle, expressed in the map frame

        Args:
            delta_pos_particle: a 3x1 numpy array expressing the delta in the particle's position [[del x], [del y], [del z = 1]]
                this is expressed in the particle's reference frame, which means the delta is also its new position, in its own reference frame
            delta_ang: the change in the particle's angle, in radians
        Returns:
            N/A
        """

        # apply transformation to delta_pos to get the new particle position in odom
        particle_position_map = np.matmul(
            self.get_transform(), delta_pos_particle)

        # update particle positions and apply the delta theta to the angle
        self.x = particle_position_map[0]
        self.y = particle_position_map[1]
        self.theta = self.theta + delta_ang


class ParticleFilter(Node):
    """ The class that represents a Particle Filter ROS Node
        Attributes list:
            base_frame: the name of the robot base coordinate frame (should be "base_footprint" for most robots)
            map_frame: the name of the map coordinate frame (should be "map" in most cases)
            odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
            scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
            n_particles: the number of particles in the filter
            d_thresh: the amount of linear movement before triggering a filter update
            a_thresh: the amount of angular movement before triggering a filter update
            pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
            particle_pub: a publisher for the particle cloud
            last_scan_timestamp: this is used to keep track of the clock when using bags
            scan_to_process: the scan that our run_loop should process next
            occupancy_field: this helper class allows you to query the map for distance to closest obstacle
            transform_helper: this helps with various transform operations (abstracting away the tf2 module)
            particle_cloud: a list of particles representing a probability distribution over robot poses
            current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                                   The pose is expressed as a list [x,y,theta] (where theta is the yaw)
            thread: this thread runs your main loop
    """

    def __init__(self):
        super().__init__('pf')
        self.base_frame = "base_footprint"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.scan_topic = "scan"        # the topic where we will get laser scans from

        self.n_particles = 300          # the number of particles to use

        # the amount of linear movement before performing an update
        self.d_thresh = 0.2
        # the amount of angular movement before performing an update
        self.a_thresh = math.pi/6

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self.update_initial_pose, 10)

        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = self.create_publisher(
            ParticleCloud, "particle_cloud", qos_profile_sensor_data)

        # laser_subscriber listens for data from the lidar
        self.create_subscription(
            LaserScan, self.scan_topic, self.scan_received, 10)

        # this is used to keep track of the timestamps coming from bag files
        # knowing this information helps us set the timestamp of our map -> odom
        # transform correctly
        self.last_scan_timestamp = None
        # this is the current scan that our run_loop should process
        self.scan_to_process = None
        # your particle cloud will go here
        self.particle_cloud = []

        self.current_odom_xy_theta = []
        self.occupancy_field = OccupancyField(self)
        self.transform_helper = TFHelper(self)

        # we are using a thread to work around single threaded execution bottleneck
        thread = Thread(target=self.loop_wrapper)
        thread.start()
        self.transform_update_timer = self.create_timer(
            0.05, self.pub_latest_transform)

    def pub_latest_transform(self):
        """ This function takes care of sending out the map to odom transform """
        if self.last_scan_timestamp is None:
            return
        postdated_timestamp = Time.from_msg(
            self.last_scan_timestamp) + Duration(seconds=0.1)
        self.transform_helper.send_last_map_to_odom_transform(
            self.map_frame, self.odom_frame, postdated_timestamp)

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        while True:
            self.run_loop()
            time.sleep(0.1)

    def run_loop(self):
        """ This is the main run_loop of our particle filter.  It checks to see if
            any scans are ready and to be processed and will call several helper
            functions to complete the processing.

            You do not need to modify this function, but it is helpful to understand it.
        """
        if self.scan_to_process is None:
            return
        msg = self.scan_to_process

        (new_pose, delta_t) = self.transform_helper.get_matching_odom_pose(self.odom_frame,
                                                                           self.base_frame,
                                                                           msg.header.stamp)
        if new_pose is None:
            # we were unable to get the pose of the robot corresponding to the scan timestamp
            if delta_t is not None and delta_t < Duration(seconds=0.0):
                # we will never get this transform, since it is before our oldest one
                self.scan_to_process = None
            return

        (r, theta) = self.transform_helper.convert_scan_to_polar_in_robot_frame(
            msg, self.base_frame)
        print("r[0]={0}, theta[0]={1}".format(r[0], theta[0]))
        # clear the current scan so that we can process the next one
        self.scan_to_process = None

        self.odom_pose = new_pose
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
            self.odom_pose)
        print("x: {0}, y: {1}, yaw: {2}".format(*new_odom_xy_theta))

        if not self.current_odom_xy_theta:
            self.current_odom_xy_theta = new_odom_xy_theta
        elif not self.particle_cloud:
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud(msg.header.stamp)
            self.update_robot_pose()
        elif self.moved_far_enough_to_update(new_odom_xy_theta):
            # we have moved far enough to do an update!
            self.move_particles_with_odom()    # update based on odometry
            self.weight_particles_with_laser(
                r, theta)   # update based on laser scan
            self.update_robot_pose()                # update robot's pose based on particles
            # resample particles to focus on areas of high density
            self.resample_particles()
        # publish particles (so things like rviz can see them)
        self.publish_particles(msg.header.stamp)

    def moved_far_enough_to_update(self, new_odom_xy_theta):
        return math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or \
            math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or \
            math.fabs(
                new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh

    def update_robot_pose(self):
        """ Update the estimate of the robot's pose given the updated particles.
            There are two logical methods for this:
                (1): compute the mean pose
                (2): compute the most likely pose (i.e. the mode of the distribution)
        """
        # first make sure that the particle weights are normalized
        self.normalize_particles()

        # TODO: assign the latest pose into self.robot_pose as a geometry_msgs.Pose object
        # just to get started we will fix the robot's pose to always be at the origin
        self.robot_pose = Pose()
        if hasattr(self, 'odom_pose'):
            self.transform_helper.fix_map_to_odom_transform(self.robot_pose,
                                                            self.odom_pose)
        else:
            self.get_logger().warn("Can't set map->odom transform since no odom data received")

    def move_particles_with_odom(self):
        """ Update the particles using the newly given odometry pose.
            The function computes the value delta which is a tuple (x,y,theta)
            that indicates the change in position and angle between the odometry
            when the particles were last updated and the current odometry.
        """
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
            self.odom_pose)
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        # create transformation matrix from odom to old neato reference frame
        old_x_odom = old_odom_xy_theta[0]
        old_y_odom = old_odom_xy_theta[1]
        old_theta_odom = old_odom_xy_theta[2]

        # applying this matrix to points in the old neato frame will return their coordinates in the odom frame
        T_old_to_odom = np.array([[cos(old_theta_odom), -sin(old_theta_odom), old_x_odom], [
                                 sin(old_theta_odom), cos(old_theta_odom), old_y_odom], [0, 0, 1]])

        # the inverse of the above matrix does the opposite
        T_odom_to_old = np.linalg.inv(T_old_to_odom)

        """
        find location of current neato position in old neato reference frame
        this is also the delta from old to new in the neato's reference frame and the particles' reference frames
        """
        current_position = np.array(
            [self.current_odom_xy_theta[0], self.current_odom_xy_theta[1], 1])

        # since the current position is expressed in odom,
        # multiplying it with T_odom_to_old will tell how far the current position is from the old position,
        # in the old reference frame
        delta_position = np.matmul(T_odom_to_old, current_position)
        delta_angle = delta[2]

        # update particles using deltas
        for particle in self.particle_cloud:
            particle.update_pose(delta_position, delta_angle)

    def resample_particles(self):
        """ 
        remove particles based on sorting function, normalize weights, and add new points around survivors
        with weight basis (output is not normalized)

        TODO: potential args to make function more adjustable: threshold input for on the fly adjustment?
        """
        # points may come in normalized by update_robot_pose

        # generate average value if each particle was equally weighted for step function
        threshold = 1/self.n_particles

        # remove particles that exceed weight threshold
        print(f"pre-resample: {len(self.particle_cloud)} particles")
        for particle in self.particle_cloud:
            if particle.w < threshold:
                self.particle_cloud.remove(particle)
        print(f"post-resample: {len(self.particle_cloud)} particles")

        if len(self.particle_cloud) == self.n_particles:
            raise Exception("no particles removed by filtering ")

        # make sure the distribution is normalized
        self.normalize_particles()

        # find information to feed to point resampling: num needed, represent respective weight
        # int, how many new particles needed
        to_generate = self.n_particles-len(self.particle_cloud)
        weights = [particle.w for particle in self.particle_cloud]

        # TODO: refactor so weights list is set up when checking particle weights against threshold

        # generate list of point objects based on likelihood from weight
        seeds = draw_random_sample(self.particle_cloud, weights, to_generate)
        for seed_particle in seeds:
            # feed parent particle to new particle as seed
            self.particle_cloud.append(Particle(parent_particle=seed_particle))

        if len(self.particle_cloud) != self.n_particles:
            raise Exception(
                "New particles generated incorrectly! Desired particle total not reached")

    def weight_particles_with_laser(self, r, theta):
        """ Updates the particle weights in response to the scan data
            r: the distance readings to obstacles
            theta: the angle relative to the robot frame for each corresponding reading 
        """

        """
        Scan data in the following section is in the neato/particle frame
        """
        # convert r list into numpy array of dimesions (len(r) x 1)
        range_vec = np.reshape(np.array(r), (-1, 1))

        # create matrix of size (len(theta) x 2), where the first col. is cos(theta[i]) and the second col. is sin(theta[i])
        theta_vec = np.reshape(np.array(theta), (-1, 1))
        cos_sin_matrix = np.concatenate(
            (np.cos(theta_vec), np.sin(theta_vec)), axis=1)

        # column-wise multiplication gives matrix of size (len(r) x 2) of the scan data in cartesian coordinates
        scan_data_cartesian = range_vec * cos_sin_matrix

        # append a column of 1s to the scan data and transpose to get a (3 x len(r)) matrix
        scan_points = np.concatenate(
            (scan_data_cartesian, np.ones((len(r), 1))), axis=1).T

        # distance to nearest point that is considered good enoughs
        step_cutoff = .1

        # iterate through each particle and update its weight based on the scan data
        for particle in self.particle_cloud:
            # express scan data in map
            scan_points_in_map = np.matmul(
                particle.get_transform(), scan_points)

            # get distance to obstacle nearest to point
            nearest_dist = self.occupancy_field.get_closest_obstacle_distance(
                x=scan_points_in_map[0], y=scan_points_in_map[1])

            # weight will be number of closest distances that are within cutoff value
            particle.w = float(sum(nearest_dist <= step_cutoff))

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
            msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)
        self.update_robot_pose()

    def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """ Initialize the particle cloud and update robot pose from cloud outputted. 
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is omitted, the odometry will be used """
        if xy_theta is None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
                self.odom_pose)
        xy_range = .8           # cartesian soft max (3x standard deviation)
        # angle radians soft max (3x standard deviation)
        theta_range = pi/2

        self.particle_cloud = []
        pose_and_ranges = {'coords': [(xy_theta[0], xy_theta[1]), xy_range], 'theta': [
            xy_theta[2], theta_range]}
        for _ in range(self.n_particles):
            self.particle_cloud.append(
                Particle(pose_and_ranges=pose_and_ranges))

        self.normalize_particles()

    def normalize_particles(self):
        """ Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
        # add up all the weights of the particles with a generator expression
        total_weight = sum(particle.w for particle in self.particle_cloud)
        # check to see if the range is already normalized
        # if abs(total_weight-1) < .01:
        #     raise Exception("This range was already normalized")
        # catch functions double-normalizing when prototyping
        # update each weight as itself divided by the total weight of the entries
        for particle in self.particle_cloud:
            particle.w = particle.w/total_weight
        # after for loop print sanity check values
        print(
            f"total weight is {sum(particle.w for particle in self.particle_cloud)}")

    def publish_particles(self, timestamp):
        msg = ParticleCloud()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = timestamp
        for p in self.particle_cloud:
            msg.particles.append(Nav2Particle(pose=p.as_pose(), weight=p.w))
        self.particle_pub.publish(msg)

    def scan_received(self, msg):
        self.last_scan_timestamp = msg.header.stamp
        # we throw away scans until we are done processing the previous scan
        # self.scan_to_process is set to None in the run_loop
        if self.scan_to_process is None:
            self.scan_to_process = msg


def main(args=None):
    rclpy.init()
    n = ParticleFilter()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
