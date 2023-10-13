# Recorded Bags

## Method

We recorded two bag files of our particle filter in action. The first is called `pf_on_mac_take_1`, and the second is called `pf_on_mac_take_2`; both are located in the `/bags` directory of this repository.

Both bag files were collected using the same method:

1. Begin running our particle filter using `ros2 launch robot_localization test_pf.py map_yaml:=path-to-your-yaml-file`, where the path to the yaml file points to the `mac_1st_floor_final.yaml` map in the `/maps` directory of this repository.
2. Launch rviz2 using `rviz2 -d ~/ros2_ws/src/robot_localization/rviz/turtlebot_bag_files.rviz`, which opens rviz2 with the correct settings for viewing the particle filter in action.
3. Begin recording our bag file using `ros2 bag record /accel /bump /odom /cmd_vel /scan /robot_description /stable_scan /projected_stable_scan /clock /tf /tf_static /particle_cloud -o pf_on_mac_take_n`. It is important to add the `/particle_cloud` topic to the recorded bag.
4. Run one of the already given bag files in the `/bags` directory of this repository (the take numbers on the recorded bags line up with the take numbers on the given bags), using `ros2 bag play path-to-your-bag-file --clock`.

Using these steps allowed us to utilize real-life data from a neato run in the MAC without having to physically work with the neatos. As a result, we are able to visualize our particle filter in action on a neato moving in the MAC.

## Discussion

From these results, it seems like our particle filter implementation works well, as the particles were able to converge on the robot's location early on in its run. Once the particles lock on to its position, they follow it for the rest of the run, largely because the odom updates at each step move them to a position that should already be where the robot is.

One difference between takes 1 and 2 is that the particles were able to identify where the robot is much faster in take 1 than take 2. This is likely because the starting location of the robot in take 2 was in a more open space, with more opportunity for many particles to have similar weightings up front, while the starting location of the robot in take 1 was in a tighter space, which can lead to greater disparity between good and bad particles right away.
