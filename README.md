# Orphaned package warning
These packages are no longer maintained and will not work with newer versions of ROS and Gazebo.
The latest ROS distribution supported is ROS2 Humble.
Feel free to fork and continue development (BSD license).

The velodyne_description package remains in ROS2 Jazzy because the community is using it, but it is not maintained.

# Velodyne Simulator
URDF description and Gazebo plugins to simulate Velodyne laser scanners

![rviz screenshot](img/rviz.png)

# Features
* URDF with colored meshes
* Gazebo plugin based on [gazebo_plugins/gazebo_ros_ray_sensor](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/foxy/gazebo_plugins/src/gazebo_ros_ray_sensor.cpp)
* Publishes PointCloud2 with same structure (x, y, z, intensity, ring, time)
* Simulated Gaussian noise
* GPU acceleration (with known issues)
* Supported models:
    * [VLP-16](velodyne_description/urdf/VLP-16.urdf.xacro)
    * [HDL-32E](velodyne_description/urdf/HDL-32E.urdf.xacro)
    * Pull requests for other models are welcome
* Experimental support for clipping low-intensity returns

# Parameters
* ```*origin``` URDF transform from parent link.
* ```parent``` URDF parent link name. Default ```base_link```
* ```name``` URDF model name. Also used as tf frame_id for PointCloud2 output. Default ```velodyne```
* ```topic``` PointCloud2 output topic name. Default ```/velodyne_points```
* ```hz``` Update rate in hz. Default ```10```
* ```lasers``` Number of vertical spinning lasers. Default ```VLP-16: 16, HDL-32E: 32```
* ```samples``` Nuber of horizontal rotating samples. Default ```VLP-16: 1875, HDL-32E: 2187```
* ```organize_cloud``` Organize PointCloud2 into 2D array with NaN placeholders, otherwise 1D array and leave out invalid points. Default ```false```
* ```min_range``` Minimum range value in meters. Default ```0.9```
* ```max_range``` Maximum range value in meters. Default ```130.0```
* ```noise``` Gausian noise value in meters. Default ```0.008```
* ```min_angle``` Minimum horizontal angle in radians. Default ```-3.14```
* ```max_angle``` Maximum horizontal angle in radians. Default ```3.14```
* ```min_intensity``` Drop points below this intensity (default: no clipping). Intensity is passed through from Gazebo `laser_retro` without modification.
* ```gpu``` Use gpu_ray sensor instead of the standard ray sensor. Default ```false```
* ```threads``` Point cloud conversion threads: ```1``` = serial (fastest for typical lidars), ```>1``` = OpenMP when point count exceeds 100k, ```0``` = auto. Default ```1```

## CPU ray intensity (`gpu:=false`)

By default (`use_distance_intensity:=true`), intensity is:

`material_retro * (intensity_reference_range / range) ^ intensity_distance_exponent`

where `material_retro` is Gazebo's per-collision `laser_retro`. Set `use_distance_intensity:=false` to pass Gazebo intensity through unchanged. `gaussian_noise` affects **range only**.

# Known Issues
* At full sample resolution, Gazebo can take up to 30 seconds to load the VLP-16 plugin, 60 seconds for the HDL-32E
* When accelerated with the GPU option, ranges are heavily quantized ([image](img/gpu.png))
    * Solution: Use CPU instead of GPU
* Gazebo cannot maintain 10Hz with large pointclouds
    * Solution: User can reduce number of points (samples) or frequency (hz) in the urdf parameters, see [example.urdf.xacro](velodyne_description/urdf/example.urdf.xacro)
* Gazebo crashes when updating HDL-32E sensors with default number of points. "Took over 1.0 seconds to update a sensor."
    * Solution: User can reduce number of points in urdf (same as above)

# Example Gazebo Robot
```ros2 launch velodyne_description example.launch.py```

# Example Gazebo Robot (with GPU)
```ros2 launch velodyne_description example.launch.py gpu:=true```

