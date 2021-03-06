# ens_vision

# Table of Contents
   * [Aruco detection](#aruco-detection)
   * [SVD transform](#svd-transform)
   * [Trajectory recorder](#trajectory-recorder)
   * [Lap detection](lap-detection)
   * [Plot trajectory errors](#plot-trajectory-errors)
   * [TF tools](#tf-tools)
      * [Save a TF](#save-a-tf)
      * [Load a TF](#load-a-tf)
   * [Path tools](#path-tools)
      * [Path type](#path-type)
      * [Save a path](#save-a-path)
      * [Load a path](#load-a-path)
      * [Convert path](#convert-path)
   
   
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

Compute the transformation T and R between the map frame and the camera frame. `pos_camera = R*pos_map + T` with pos a column vector x,y,z. Python file `svd_camera.py`, points in the map frame are collected with the `Publish Point` tool of RViz.

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

# Lap detection 

* Generate a finish line between two given points on the static_frame_id
* Detect when moving_frame_id crosses this line
* Send a message on `/syscommand` topic: "lap"

```bash
rosrun ens_vision line_crossing.py static_frame_id moving_frame_id
```

### Required tf transforms

* static_frame_id (default *map*) -> moving_frame_id (default *marker_0*)

### Published topics

* /finish_line
* /syscommand

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
* `ax_mps2`: float32, meter/second??. Target acceleration in current point. We assume this acceleration to be constant
  from current point until next point.
* `s_m`: float32, meter. Curvi-linear distance along the raceline. **This data is moved to seventh column isntead of the first one**

## Save a path

Save a ROS Path with the python function :
```python
filename = path_tools.save_path(ros_path, name='path', test=False)
```

Or save a MCP path with the python function :
```python
filename = path_tools.save_mcp(mcp_path)
```

By default the files are saved in the `/paths` but it can be changed to the `/tests` directory with the argument *test*.

## Load a path

Load a ROS path with this python function :
```python
ros_path = load_path(name, absolute_path=False)
```
`name` can either be the filename (without the extension) or the absolute path of the file if `absolute_path`. By default the files are opened from the `/paths` directory.

Load a MCP path with this python function :
```python
mcp_path = load_mcp(name)
```

## Convert path

### MCP to ROS
```python
ros_path = path_tools.mcp_to_path(mcp_path)
```
This function can also be used to transform a [x,y,yaw] array to ROS path.

### X,Y to ROS
Compute the yaw along the path and create a ROS path from X and Y coordinates
```python
ros_path = path_tools.xy_to_path(X,Y)
```

### ROS to X,Y, Yaw, t 
Generate X, Y and Yaw list (+ time list if pose are timestamped and `time')
```python
X, Y, Yaw (, t) = path_tools.path_to_xyyaw(ros_path,time=False)
```
