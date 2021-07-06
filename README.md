# Providing Odom data with laser_scan_matcher package

This repository is an answer to this [question](https://www.reddit.com/r/ROS/comments/ocmfgo/please_help_me/) at reddit.

## Answer

### Generating data from LaserScan data

Install [scan_tools](http://wiki.ros.org/scan_tools) package

    sudo apt-get install ros-DISTRO-scan-tools

Create a launch file

    <launch>
      <node pkg="laser_scan_matcher" type="laser_scan_matcher_node" name="lsm_node" output="screen">
	     <param name="fixed_frame" value="map"/>
		   <param name="base_frame" value="base_link"/>
		   <param name="use_imu" value="false"/>
		   <param name="use_odom" value="false"/>
		   <param name="publish_tf" value="false"/>
		   <param name="publish_pose" value="false"/>
		   <param name="publish_pose_stamped" value="true"/>
      </node>
    </launch>

| Parameter | Type | Value | Description |
|--|--|--|--|
| fixed_frame | string | map | Robot's fixed frame, eg. map, world, odom |
| base_frame | string | base_link | Robot's base frame, eg. base_link, base_footprint |
| use_imu | bool | false | Whether to use an imu for the theta prediction of the scan registration. Requires input on  /imu/data  topic |
| use_odom | bool | false | Whether to use wheel odometry for the x-, y-, and theta prediction of the scan registration. Requires input on  odom  topic |
| publish_tf | bool | false | Whether to publish scan matcher's estimation for the position of the base frame in the world frame as a transform |
| publish_pose | bool | false | Whether to publish scan matcher's estimation for the position of the base frame in the world frame as a  [geometry_msgs/Pose2D](http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose2D.html) |
| publish_pose_stamped| bool | true| Whether to publish scan matcher's estimation for the position of the base frame in the world frame as a  [geometry_msgs/PoseStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html) |

LaserScan data **MUST** be published to **topic /scan**. 
For more details, read the [wiki](http://wiki.ros.org/laser_scan_matcher)

### Converting data from [geometry_msgs/PoseStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html) and publish to /odom

Install [robot_localization](http://wiki.ros.org/robot_localization) package

    sudo apt-get install ros-DISTRO-robot-localization
    
Create a launch file

       <launch>
    	 <node pkg="robot_localization" type="ukf_localization_node" name="ukf" clear_params="true" output="screen">
    	    <remap from="odometry/filtered" to="odom"/>
    	    <param name="map_frame" value="map"/>
    	    <param name="odom_frame" value="odom"/>
    	    <param name="base_link_frame" value="base_link"/>
    	    <param name="world_frame" value="odom"/>
    	    <param name="two_d_mode" value="true"/>
    	    <param name="pose0" value="/pose_with_covariance_stamped"/>
    	    <param name="pose0_config" value="[true, true, false, false, false, true, false, false, false, false, false, false, false, false, false]"/>
    	 </node>
    	</launch>
    	
| Parameter | Type | Value | Description |
|--|--|--|--|
| map_frame | string | map | Robot's map frame, eg. map, world |
| odom_frame | string | odom| Robot's odom frame, eg. odom |
| base_link_frame | string | base_link| Robot's base frame, eg. base_link, base_footprint |
| world_frame| string | odom| Robot's odom frame, eg. odom |
| two_d_mode | bool | true| Set as true if your robot doesn't move on Z axis |
| pose0| string | /pose_with_covariance_stamped| Topic that the laser_scan_matcher is publishing the pose [geometry_msgs/Pose2D](http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose2D.html) |
| pose0_config| bool[] | [true, true, false, false, false, true, false, false, false, false, false, false, false, false, false] | Array of booleans to set which values of the pose may be considered to the  calculations |

For more details, read the [repo](https://github.com/cra-ros-pkg/robot_localization)

### Finally

 1. Run the scanner/lidar node
 2. Run AMCL node
 3. Launch laser_scan_matcher file
 4. Launch ukf file

The laser_scan_matcher pose has no use to the user, but it may be finded at the follow topic

    /pose_with_covariance_stamped

   The odom data may be finded at the follow topic
   
    /odom
