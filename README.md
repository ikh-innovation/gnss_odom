# GNSS Odometry ROS Package

This ROS package computes odometry heading for a robot using GNSS (Global Navigation Satellite System) data or wheel odometry data. It is designed to work with ROS Melodic and provides a modular and configurable implementation.

---

## **Table of Contents**
1. [Overview](#overview)
2. [Features](#features)
3. [Calculation](#calculation)
3. [Dependencies](#dependencies)
4. [Installation](#installation)
5. [Usage](#usage)
6. [Parameters](#parameters)
7. [Launch Files](#launch-files)


---

## **Overview**

The `gnss_odom` package computes odometry for a robot using either:
- **GNSS data**: Computes heading and speed based on consecutive GNSS fixes.
- **Wheel odometry data**: Computes heading and speed based on displacement from wheel encoders.

The package publishes the computed odometry as an `Odometry` message, which can be used by other ROS nodes for navigation, localization, or mapping.

---

## **Features**
- Computes heading from GNSS fixes or odometry data.
- Uses least-squares fitting for a smoother heading estimate.
- Corrects heading direction based on velocity sign.
- Publishes odometry messages with orientation information.
---
## **Calculation**

The node supports two methods for heading calculation:

1. **Direct GNSS Bearing**
   - Uses `pyproj.Geod.inv()` to get the GNSS bearing.
   - Converts it to radians and applies an optional offset.

2. **Least-Squares Line Fitting**
   - Stores recent GNSS/odometry positions.
   - Fits a line using `np.linalg.lstsq()`.
   - Computes the heading angle from the line slope.
   - Corrects direction based on movement and velocity.

---

## **Dependencies**

### ROS Packages
- `rospy`
- `sensor_msgs`
- `nav_msgs`
- `geometry_msgs`

### Python Libraries
- `pyproj` (for geodetic calculations)
- `numpy` (for basic mathematical operations)

---

## **Installation**

1. **Clone the Repository**:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/ikh-innovation/gnss_odom.git
   ```

2. **Install Dependencies**:
   Ensure that the required ROS packages are installed:
   ```bash
   sudo apt-get install ros-melodic-sensor-msgs ros-melodic-nav-msgs ros-melodic-geometry-msgs
   ```
   Install the `pyproj` Python library:
   ```bash
   pip install pyproj numpy
   ```

3. **Build the Package**:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

---

## **Usage**

### Running the Node
To run the `gnss_odom` node, use the following command:
```bash
rosrun gnss_odom gnss_odom
```

### Launch Files
The package includes launch files for easy configuration and execution. To use a launch file:
```bash
roslaunch gnss_odom gnss_odom.launch
```

---

## **Parameters**

The following parameters can be configured for the `gnss_odom` node:

| Parameter Name        | Description | Default Value |
|----------------------|-------------|--------------|
| `cmd_vel_topic`      | Command velocity topic | `/aristos/wheel/cmd_vel` |
| `fix_topic`          | GNSS fix topic | `/aristos/filter/positionlla/gnss` |
| `odom_topic`         | Odometry input topic | `/aristos/odometry/gps/onlyRTK` |
| `odom_pub_topic`     | Published odometry topic | `gnss_heading/odom` |
| `use_odometry`       | Use odometry instead of GNSS for heading | `true` |
| `velocity_threshold` | Minimum velocity for heading update | `0.2` m/s |
| `distance_threshold` | Minimum distance for heading update | `0.05` m |
| `heading_offset`     | Additional heading offset (radians) | `0.0` |
| `use_fitted_heading` | Enable least-squares heading estimation | `true` |
| `num_fit_points`     | Number of past points used for fitting | `4` |
| `gnss_ellipsoid`     | Ellipsoid model used by pyproj | `WGS84` |

---

### **Published Topics**
- **`gnss_heading/odom`** (`nav_msgs/Odometry`)  
  Publishes computed odometry data with orientation.

### **Subscribed Topics**
- **`/aristos/wheel/cmd_vel`** (`geometry_msgs/Twist`)  
  Stores velocity data for heading correction.
- **`/aristos/filter/positionlla/gnss`** (`sensor_msgs/NavSatFix`)  
  Processes GNSS data to compute heading.
- **`/aristos/odometry/gps/onlyRTK`** (`nav_msgs/Odometry`)  
  Processes odometry data when enabled.

## **Launch Files**

The package includes the following launch files:

### `gnss_odom.launch`
Launches the `gnss_odom` node with default parameters. You can override parameters using the `arg` tag.

Example:
```xml
<launch>

  <node name="gnss_odom_node" pkg="gnss_odom" type="gnss_odom" output="screen">
    <param name="cmd_vel_topic" value="/aristos/wheel/cmd_vel" />
    <param name="fix_topic" value="/aristos/filter/positionlla/gnss" />
    <param name="odom_topic" value="/aristos/odometry/gps/onlyRTK"/>
    <param name="odom_pub_topic" value="gnss_heading/odom" />
    <param name="use_odometry" value="true"/>
    <param name="velocity_threshold" value="0.2"/>
    <param name="distance_threshold" value="0.05"/>
    <param name="heading_offset" value="0.0"/>
    <param name="use_fitted_heading" value="true"/>
    <param name="num_fit_points" value="4"/>
    <param name="gnss_ellipsoid" value="WGS84"/> <!--pyproj supported ellipsoids-->
  </node>

</launch>
```



