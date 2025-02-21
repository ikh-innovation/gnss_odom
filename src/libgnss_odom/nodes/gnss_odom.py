import rospy
import math
import numpy as np
from pyproj import Geod
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from collections import deque

class Quaternion:
    def __init__(self, w=0.0, x=0.0, y=0.0, z=0.0):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    @staticmethod
    def from_euler(roll=0.0, pitch=0.0, yaw=0.0):
        # Convert Euler angles to Quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        return Quaternion(
            w=cy * cp * cr + sy * sp * sr,
            x=cy * cp * sr - sy * sp * cr,
            y=sy * cp * sr + cy * sp * cr,
            z=sy * cp * cr - cy * sp * sr
        )

class GNSSOdometry:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('gnss_odom', anonymous=True)
        
        # Retrieve parameters from the parameter server
        self.use_odometry = rospy.get_param('~use_odometry', False)
        self.use_velocity_criteria = rospy.get_param('~use_velocity_criteria', True)
        self.velocity_linear_threshold = rospy.get_param('~velocity_linear_threshold', 0.03)
        self.velocity_angular_threshold = rospy.get_param('~velocity_angular_threshold', 0.5)
        self.lower_distance_threshold = rospy.get_param('~lower_distance_threshold', 0.05)
        self.upper_distance_threshold = rospy.get_param('~upper_distance_threshold', 0.2)
        self.initial_covariance = rospy.get_param('~initial_covariance', 0.1)
        self.heading_offset = rospy.get_param('~heading_offset', 0.0)
        self.heading_diff_publish_ths = rospy.get_param('~heading_diff_publish_ths', 0.34)
        self.gnss_ellipsoid = rospy.get_param('~gnss_ellipsoid', 'WGS84')
        self.prev_fix = None
        self.prev_cmd = None
        self.prev_odom = None
        self.last_published_time = rospy.get_time()
        
        # Line fitting feature
        self.use_fitted_heading = rospy.get_param('~use_fitted_heading', False)
        self.num_fit_points = rospy.get_param('~num_fit_points', 5)
        if self.num_fit_points < 3:
            self.num_fit_points = 2
            rospy.logwarn("Default number of fit points set: {}".format())

        self.timeout = rospy.get_param('~timeout', self.num_fit_points * 1.0 / 5.0 + 1)  # num_points * topic_hz + 1
        
        self.fit_points = deque(maxlen=self.num_fit_points) 

        # Publish cmd filtering
        self.published_headings_length = rospy.get_param('~published_headings_length', 5)
        self.published_headings = deque(maxlen=self.published_headings_length) 
        
        self.geod = Geod(ellps=self.gnss_ellipsoid)

        # Publisher for odometry data
        self.odom_pub = rospy.Publisher(
            rospy.get_param('~odom_pub_topic', 'gnss/odom'),
            Odometry,
            queue_size=1
        )

        # Subscriber for command velocity
        rospy.Subscriber(
            rospy.get_param('~cmd_vel_topic', 'husky_velocity_controller/cmd_vel'),
            Twist,
            self.store_cmd_vel
        )
        
        # Subscriber for odometry or GNSS fix data
        if self.use_odometry:
            rospy.Subscriber(
                rospy.get_param('~odom_topic', 'odometry/data'),
                Odometry,
                self.odom_callback
            )
        else:
            rospy.Subscriber(
                rospy.get_param('~fix_topic', 'gnss/fix'),
                NavSatFix,
                self.gnss_callback
            )

    def store_cmd_vel(self, cmd_data):
        # Store the latest command velocity
        self.prev_cmd = cmd_data

    def compute_fitted_heading(self):
        """Compute heading using line fitting and correct it based on robot motion."""
        if len(self.fit_points) < 2:
            return None, self.initial_covariance

        # Extract x and y coordinates
        fit_points_array = np.array(self.fit_points)
        x_vals = fit_points_array[:, 0]
        y_vals = fit_points_array[:, 1]

        # Least Squares Fitting
        A = np.vstack([x_vals, np.ones(len(x_vals))]).T
        m, b = np.linalg.lstsq(A, y_vals, rcond=None)[0]  # Solve y = mx + b
        fitted_heading = math.atan(m)  # Convert slope to angle

        # Compute Residuals for Covariance
        y_predicted = m * x_vals + b
        residuals = y_vals - y_predicted
        covariance = np.var(residuals) if len(residuals) > 1 else self.initial_covariance

        # Correct Heading Based on Robot Motion
        x_first, y_first = self.fit_points[0]
        x_last, y_last = self.fit_points[-1]
        dx = x_last - x_first
        dy = y_last - y_first

        # Compute the displacement angle
        displacement_angle = math.atan2(dy, dx)

        # Check if the fitted heading aligns with the displacement direction
        angle_diff = abs(fitted_heading - displacement_angle)
        if angle_diff > math.pi / 2:  # Fitted heading is opposite to displacement direction
            fitted_heading += math.pi  # Flip by 180 degrees

        # Apply velocity sign correction
        if self.prev_cmd.linear.x < 0:  # Robot moving backward
            fitted_heading += math.pi  # Flip by 180 degrees

        # Wrap heading to [-pi, pi]
        fitted_heading = (fitted_heading + math.pi) % (2 * math.pi) - math.pi

        return fitted_heading, covariance

    def compute_odom_from_odometry(self, odom_data):
        if self.prev_odom is not None and self.prev_cmd is not None:
            if not self.use_velocity_criteria or (abs(self.prev_cmd.linear.x) >= self.velocity_linear_threshold and abs(self.prev_cmd.angular.z) <= self.velocity_angular_threshold):
                # Compute distance moved
                dx = odom_data.pose.pose.position.x - self.prev_odom.pose.pose.position.x
                dy = odom_data.pose.pose.position.y - self.prev_odom.pose.pose.position.y
                distance = math.sqrt(dx**2 + dy**2)
                heading = None

                if self.lower_distance_threshold <= distance <= self.upper_distance_threshold:
                    if self.use_fitted_heading:
                        self.fit_points.append((odom_data.pose.pose.position.x, odom_data.pose.pose.position.y))
                        if len(self.fit_points) >= self.num_fit_points:
                            heading, covariance = self.compute_fitted_heading()
                    else:
                        heading = math.atan2(dy, dx)
                        covariance = self.initial_covariance
                    
                    if heading is not None:
                        q = Quaternion.from_euler(0.0, 0.0, heading + self.heading_offset)
                        
                        odom_data.pose.pose.orientation.x = q.x
                        odom_data.pose.pose.orientation.y = q.y
                        odom_data.pose.pose.orientation.z = q.z
                        odom_data.pose.pose.orientation.w = q.w
                        
                        # Add computed covariance to odom_data.pose.covariance
                        odom_data.pose.covariance[0] = covariance
                        odom_data.pose.covariance[7] = covariance
                        odom_data.pose.covariance[14] = covariance
                        odom_data.pose.covariance[21] = covariance
                        odom_data.pose.covariance[28] = covariance
                        odom_data.pose.covariance[35] = covariance
                        
                        return odom_data, heading
                    
                elif distance < self.lower_distance_threshold:
                    pass
                else:
                    self.last_published_time = rospy.get_time()
                    self.fit_points.clear()

        self.prev_odom = odom_data
        return None, None

    def publish_odom(self, odom_data, heading):
        if odom_data is not None and heading is not None:
            if len(self.published_headings) >= self.published_headings_length:
                # Filtering out spikes
                if abs(np.mean(self.published_headings) - (heading + self.heading_offset)) < self.heading_diff_publish_ths:
                    self.odom_pub.publish(odom_data)
            self.published_headings.append(heading + self.heading_offset)

    def odom_callback(self, odom_data):
        odom_data, heading = self.compute_odom_from_odometry(odom_data)
        self.publish_odom(odom_data, heading)

    def gnss_callback(self, fix_data):
        # Placeholder for GNSS callback logic
        pass

    def run(self):
        # Keep the node running
        rospy.spin()

def main():
    # Create and run the GNSSOdometry node
    gnss_odom = GNSSOdometry()
    gnss_odom.run()