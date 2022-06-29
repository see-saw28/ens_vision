# ens_vision

# Table of Contents
   * [Aruco detection](#aruco-detection)
   * [SVD transform](#svd-transform)
   * [Trajectory recorder](#trajectory-recorder)
   * [Plot trajectory errors](#plot-trajectory-errors)
   
   
# Aruco detection

## Realsense D435

### Installation

https://github.com/IntelRealSense/realsense-ros/blob/development/README.md#installation-instructions


```bat
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
```
et l'installation du sdk
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages

### Published topics

* /camera/depth/image_rect_raw
* /camera/infra1/image_rect_raw
* /camera/infra1/camera_info


## Aruco marker

### Subscribed topics

* /camera/depth/image_rect_raw
* /camera/infra1/image_rect_raw
* /camera/infra1/camera_info

### Published topics

* /aruco
* /amcl_pose

### Parameters

### Provided tf transforms

* camera -> marker_{ID}

# SVD transform

### Subscribed topics

* /clicked_point

### Required tf transforms

* camera -> marker_{ID}

### Provided tf transforms

* camera -> map

### Save data

Save the tf in a file to open it later, cf [TF tools](#tf-tools)

# Trajectory recorder

### Subscribed topics

* /syscommand

### Published topics

* /trajectory
* /trajectory_loaded
* /current_lap_path
* /testing_lap_path
* /lap_{lap_number}_path

### Required tf transforms

* map -> marker_0
* map -> marker_1

or

* map -> base_link

### Save data

Save the traj on files to open it later. And save all the information for the test.

# Plot trajectory errors

## Load data

Load the trajectory files to compute the error

# TF tools

## Requirement

* Pickle

## Save a TF 

Python script :
```bash
rosrun ens_vision save_tf.py child_frame parent_frame
```
Or python function :

Need the position vector and the quaternion :
```python
tl = tf.TransformListener()
pos,quat = tl.lookupTransform(child_frame, parent_frame, rospy.Time())
```
Then save a pickle file with [pos, quat, child_frame, parent_frame]

```python
tf_tools.save_tf(pos, quat,child_frame,parent_frame)
```


## Load a TF

Python script :
```bash
rosrun ens_vision tf_tools tf_name
```

Or python function :

Load a pickle file with [pos, quat, child_frame, parent_frame] :
```python
tf_tools.load_tf(name, absolute_path=False, rate=100)
```
then publish the tf with :
```python
br = tf.TransformBroadcaster()
br.sendTransform(pos,quat,rospy.Time.now(),child_frame, parent_frame)
```



# Path tools

## Requirement

* Pickle

## Path type

### Path (ROS)

ROS message type [Doc](http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/Path.html)

### MCP path (numpy)

This type of path is computed by this [repo](https://github.com/TUMFTM/global_racetrajectory_optimization).

The seven columns are structured as follows:

* `x_m`: float32, meter. X-coordinate of raceline point.
* `y_m`: float32, meter. Y-coordinate of raceline point.
* `psi_rad`: float32, rad. Heading of raceline in current point from -pi to +pi rad. **Zero is along x-axis** (instead of y-axis like TUMFTM).
* `kappa_radpm`: float32, rad/meter. Curvature of raceline in current point.
* `vx_mps`: float32, meter/second. Target velocity in current point.
* `ax_mps2`: float32, meter/second². Target acceleration in current point. We assume this acceleration to be constant
  from current point until next point.
* `s_m`: float32, meter. Curvi-linear distance along the raceline.

