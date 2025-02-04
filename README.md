# GNSS Odometry ROS Package

This ROS package computes odometry heading for a robot using GNSS (Global Navigation Satellite System) data or wheel odometry data. It is designed to work with ROS Melodic and provides a modular and configurable implementation.

---

## **Table of Contents**
1. [Overview](#overview)
2. [Dependencies](#dependencies)
3. [Installation](#installation)
4. [Usage](#usage)
5. [Parameters](#parameters)
6. [Launch Files](#launch-files)


---

## **Overview**

The `gnss_odom` package computes odometry for a robot using either:
- **GNSS data**: Computes heading and speed based on consecutive GNSS fixes.
- **Wheel odometry data**: Computes heading and speed based on displacement from wheel encoders.

The package publishes the computed odometry as an `Odometry` message, which can be used by other ROS nodes for navigation, localization, or mapping.

---

## **Dependencies**

### ROS Packages
- `rospy`
- `sensor_msgs`
- `nav_msgs`
- `geometry_msgs`

### Python Libraries
- `pyproj` (for geodetic calculations)
- `math` (for basic mathematical operations)

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
   pip install pyproj
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

| Parameter               | Default Value                          | Description                                                                 |
|-------------------------|----------------------------------------|-----------------------------------------------------------------------------|
| `~use_odometry`         | `False`                                | If `True`, use wheel odometry data instead of GNSS data.                    |
| `~velocity_threshold`   | `0.03`                                 | Minimum linear velocity (m/s) to compute odometry.                          |
| `~distance_threshold`   | `0.5`                                  | Minimum distance (m) between consecutive fixes to compute odometry.         |
| `~initial_covariance`   | `0.1`                                  | Initial covariance value for the odometry message.                          |
| `~cmd_vel_topic`        | `husky_velocity_controller/cmd_vel`    | Topic for velocity commands (`Twist` messages).                             |
| `~fix_topic`            | `gnss/fix`                             | Topic for GNSS fixes (`NavSatFix` messages).                                |
| `~odom_topic`           | `odometry/data`                        | Topic for wheel odometry data (`Odometry` messages).                        |
| `~odom_pub_topic`       | `gnss/odom`                            | Topic to publish computed odometry (`Odometry` messages).                   |

---

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
    <param name="velocity_threshold" value="0.1"/>
    <param name="distance_threshold" value="0.075"/>
    <param name="heading_offset" value="0.0"/>
  </node>
</launch>
```

