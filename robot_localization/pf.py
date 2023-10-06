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
from math import sin, cos
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

    def __init__(self, parent_particle=None, x=0.0, y=0.0, theta=0.0, w=1.0):
        """ Construct a new Particle
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of KeyboardInterruptthe hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized """
        if parent_particle:
            x = parent_particle.x
            x = parent_particle.x
            theta = parent_particle.theta
            w = parent_particle.w

        # TODO: add variance to these values so that particles are not spawned directly on top of the point we ask them to be
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        q = quaternion_from_euler(0, 0, self.theta)
        return Pose(position=Point(x=self.x, y=self.y, z=0.0),
                    orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

    # TODO: define additional helper functions if needed
    def update_pose(self, delta_pos_particle, delta_ang):
        """
        Using the change in the particle's position, expressed in its own reference frame, 
        and the change in the particle's angle, 
        update the position of the particle, expressed in the odom frame

        Args:
            delta_pos_particle: a 3x1 numpy array expressing the delta in the particle's position [[del x], [del y], [del z = 1]]
                this is expressed in the particle's reference frame, which means the delta is also its new position, in its own reference frame
            delta_ang: the change in the particle's angle, in radians
        Returns:
            N/A
        """

        # set up tranform matrix from particle reference frame to odom
        T_particle_to_odom = np.array([[cos(self.theta), -sin(self.theta), self.x],
                                       [sin(self.theta), cos(self.theta), self.y], [0, 0, 1]])

        # apply transformation to delta_pos to get the new particle position in odom
        particle_position_odom = np.matmul(
            T_particle_to_odom, delta_pos_particle)

        # update particle positions and apply the delta theta to the angle
        self.x = particle_position_odom[0]
        self.y = particle_position_odom[1]
        self.theta = self.theta + delta_ang

    def update_weight(self, ranges, thetas, occupancy_field):
        # parameters for gaussian and also cutoff function
        mean = 0
        sigma = 0.39899
        step_cutoff = .1

        # transformation matrix from particle reference frame to odom
        T_particle_to_odom = np.array([[cos(self.theta), -sin(self.theta), self.x],
                                       [sin(self.theta), cos(self.theta), self.y], [0, 0, 1]])

        gauss_prod = 1
        close_points = 0.0
        for i in range(0, len(ranges), 90):
            # make sure scan is valid, else just don't use the point
            if ranges[i] is not None and ranges[i] != 0:
                # convert polar point to cartesian, in particle frame
                x = ranges[i] * cos(thetas[i])
                y = ranges[i] * sin(thetas[i])

                # package points into an array
                point_in_particle = np.array([[x], [y], [1]])

                # express point in odom
                point_in_odom = np.matmul(
                    T_particle_to_odom, point_in_particle)

                # get distance to obstacle nearest to point
                nearest_dist = float(occupancy_field.get_closest_obstacle_distance(
                    x=point_in_odom[0], y=point_in_odom[1]))

                if not math.isnan(nearest_dist):
                    gauss_prod *= norm.pdf(nearest_dist, mean, sigma)

                    if nearest_dist <= step_cutoff:
                        close_points += 1

        self.w = close_points

        print("Particle Position:", [self.x, self.y],
              "\n", "Particle Weight (Gauss):", gauss_prod, "\n", "Particle Weight (Step):", close_points)


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

        self.n_particles = 10          # the number of particles to use

        # the amount of linear movement before performing an update
        self.d_thresh = 0.2
        # the amount of angular movement before performing an update
        self.a_thresh = math.pi/6

        # TODO: define additional constants if needed

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
        elif self.moved_far_enough_to_update(new_odom_xy_theta):
            # we have moved far enough to do an update!
            self.update_particles_with_odom()    # update based on odometry
            self.update_particles_with_laser(
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

    def update_particles_with_odom(self):
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

        # TODO: modify particles using delta

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
        to_generate = self.n_particles-len(self.particle_cloud)
        weights = [particle.w for particle in self.particle_cloud]

        # generate list of point objects based on likelihood from weight
        seeds = draw_random_sample(self.particle_cloud, weights, to_generate)
        for seed_particle in seeds:
            # feed parent particle to new particle as seed
            self.particle_cloud.append(Particle(parent_particle=seed_particle))

        if len(self.particle_cloud) != self.n_particles:
            raise Exception(
                "New particles generated incorrectly! Desired particle total not reached")

    def update_particles_with_laser(self, r, theta):
        """ Updates the particle weights in response to the scan data
            r: the distance readings to obstacles
            theta: the angle relative to the robot frame for each corresponding reading 
        """
        # TODO: implement this

        for particle in self.particle_cloud:
            particle.update_weight(r, theta, self.occupancy_field)

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
            msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)

    def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """ Initialize the particle cloud.
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is omitted, the odometry will be used """
        if xy_theta is None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
                self.odom_pose)
        xy_range = .3           # cartesian standard deviation
        theta_range = 7        # angle degrees standard deviation

        self.particle_cloud = []
        for i in range(self.n_particles):
            self.particle_cloud.append(Particle(x=random.gauss(xy_theta[0], xy_range),
                                                y=random.gauss(
                                                    xy_theta[1], xy_range),
                                                theta=random.gauss(xy_theta[2], theta_range), w=1.0))
        self.particle_cloud.append(Particle())
        # TODO create particles

        self.normalize_particles()
        self.update_robot_pose()

    def normalize_particles(self):
        """ Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
        # add up all the weights of the particles with a generator expression
        total_weight = sum(particle.w for particle in self.particle_cloud)
        # check to see if the range is already normalized
        if abs(total_weight-1) < .01:
            raise Exception("This range was already normalized")
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
