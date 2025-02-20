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
        rospy.init_node('gnss_odom', anonymous=True)
        
        self.use_odometry = rospy.get_param('~use_odometry', False)
        self.velocity_threshold = rospy.get_param('~velocity_threshold', 0.03)
        self.lower_distance_threshold = rospy.get_param('~lower_distance_threshold', 0.05)
        self.upper_distance_threshold = rospy.get_param('~upper_distance_threshold', 0.2)
        self.initial_covariance = rospy.get_param('~initial_covariance', 0.1)
        self.heading_offset = rospy.get_param('~heading_offset',0.0)
        self.heading_diff_publish_ths = rospy.get_param('~heading_diff_publish_ths',0.34)
        self.gnss_ellipsoid = rospy.get_param('~gnss_ellipsoid','WGS84')
        self.prev_fix = None
        self.prev_cmd = None
        self.prev_odom = None
        self.last_published_time = rospy.get_time()
        
        # Line fitting feature
        self.use_fitted_heading = rospy.get_param('~use_fitted_heading', False)
        self.num_fit_points = rospy.get_param('~num_fit_points', 5)
        if (self.num_fit_points<3):
            self.num_fit_points = 2
            rospy.logwarn("Default number of fit points set: {}".format())

        self.timeout = rospy.get_param('~timeout', self.num_fit_points*1.0/5.0 + 1) #num_points*topic_hz + 1
        
        self.fit_points = deque(maxlen=self.num_fit_points) 

        # Publish cmd filtering
        self.published_headings_length = rospy.get_param('~published_headings_length', 5)
        self.published_headings = deque(maxlen=self.published_headings_length) 
        
        self.geod = Geod(ellps=self.gnss_ellipsoid)

        self.odom_pub = rospy.Publisher(
            rospy.get_param('~odom_pub_topic', 'gnss/odom'),
            Odometry,
            queue_size=1
        )

        rospy.Subscriber(
            rospy.get_param('~cmd_vel_topic', 'husky_velocity_controller/cmd_vel'),
            Twist,
            self.store_cmd_vel
        )
        
        if self.use_odometry:
            rospy.Subscriber(
                rospy.get_param('~odom_topic', 'odometry/data'),
                Odometry,
                self.compute_odom_from_odometry
            )
        else:
            rospy.Subscriber(
                rospy.get_param('~fix_topic', 'gnss/fix'),
                NavSatFix,
                self.compute_odom_from_gnss
            )

    def store_cmd_vel(self, cmd_data):
        self.prev_cmd = cmd_data

    def compute_fitted_heading(self):
        """Compute heading using line fitting and correct it based on robot motion."""
        if len(self.fit_points) < 2:
            # self.fit_points = []
            return None, self.initial_covariance

        # Extract x and y coordinates
        x_vals = np.array([p[0] for p in self.fit_points])
        y_vals = np.array([p[1] for p in self.fit_points])

        # ----- Least Squares Fitting -----
        A = np.vstack([x_vals, np.ones(len(x_vals))]).T
        m, b = np.linalg.lstsq(A, y_vals, rcond=None)[0]  # Solve y = mx + b
        fitted_heading = math.atan(m)  # Convert slope to angle

        # ----- Compute Residuals for Covariance -----
        y_predicted = m * x_vals + b
        residuals = y_vals - y_predicted
        covariance = np.var(residuals) if len(residuals) > 1 else self.initial_covariance

        # ----- Correct Heading Based on Robot Motion -----
        # Determine the direction of motion using the first and last points
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
        # self.fit_points = []

        return fitted_heading, covariance

    # def compute_odom_from_gnss(self, fix_data):
    #     print("Fit point list len: {}".format(len(self.fit_points)))
    #     if self.prev_fix is not None and self.prev_cmd is not None:
    #         if self.prev_cmd.linear.x >= self.velocity_threshold: 
    #             heading_, _, distance = self.geod.inv( self.prev_fix.longitude, self.prev_fix.latitude, fix_data.longitude, fix_data.latitude)
    #             heading = None
    #             if distance >= self.distance_threshold:

    #                 if (self.use_fitted_heading):
    #                     x, y, _ = self.geod.fwd(
    #                     self.prev_fix.longitude, self.prev_fix.latitude,
    #                     fix_data.longitude, fix_data.latitude
    #                     )
    #                     self.fit_points.append((x, y))
    #                     if len(self.fit_points)>self.num_fit_points:
    #                         self.fit_points.pop(0)
    #                         heading, covariance = self.compute_fitted_heading()
    #                         print("TAKIS")
    #                 else:
    #                     heading = math.radians(heading_)
    #                     covariance = self.initial_covariance

    #                 print("PRINT")
    #                 if (heading != None):
    #                     print("Heading: {} | Cov: {}".format(heading,covariance))
    #                     q = Quaternion.from_euler(0.0, 0.0, heading+self.heading_offset)
                        
    #                     current_odom = Odometry()
    #                     current_odom.header.stamp = fix_data.header.stamp
    #                     current_odom.header.frame_id = fix_data.header.frame_id
    #                     current_odom.pose.pose.orientation.x = q.x
    #                     current_odom.pose.pose.orientation.y = q.y
    #                     current_odom.pose.pose.orientation.z = q.z
    #                     current_odom.pose.pose.orientation.w = q.w
    #                     current_odom.pose.covariance[35] = covariance
                        
    #                     self.odom_pub.publish(current_odom)
    #             else:
    #                 self.fit_points = []

    #     self.prev_fix = fix_data

    def compute_odom_from_odometry(self, odom_data):
        # print("Fit point list len: {}".format(len(self.fit_points)))

        # fit_points_yaw = []
        # for pair in np.diff(self.fit_points, axis=0):
        #     fit_points_yaw.append(math.atan2(pair[0],pair[1]))
        # path_max_knee = max(abs(np.diff(fit_points_yaw)))

        if self.prev_odom is not None and self.prev_cmd is not None:
            print("CHECKING =====")
            if abs(self.prev_cmd.linear.x) >= self.velocity_threshold:
                # Compute distance moved
                dx = odom_data.pose.pose.position.x - self.prev_odom.pose.pose.position.x
                dy = odom_data.pose.pose.position.y - self.prev_odom.pose.pose.position.y
                distance = math.sqrt(dx**2 + dy**2)
                heading = None

                print("Distance {}".format(distance))
                current_time = rospy.get_time()
                if (distance <= self.upper_distance_threshold):
                    if (distance >= self.lower_distance_threshold): #or (current_time-self.last_published_time > self.timeout):
                        if (self.use_fitted_heading):
                            self.fit_points.append((odom_data.pose.pose.position.x, odom_data.pose.pose.position.y))
                            print("Fit points {}".format(self.fit_points))
                            if len(self.fit_points) >= self.num_fit_points:
                                heading, covariance = self.compute_fitted_heading()
                        else:
                            heading = math.atan2(dy, dx)
                            covariance = self.initial_covariance
                        
                        print("Heading {}".format(heading))
                        if heading!=None :
                            # print("Heading: {} | Cov: {}".format(heading,covariance))
                            q = Quaternion.from_euler(0.0, 0.0, heading+self.heading_offset)
                            
                            odom_data.pose.pose.orientation.x = q.x
                            odom_data.pose.pose.orientation.y = q.y
                            odom_data.pose.pose.orientation.z = q.z
                            odom_data.pose.pose.orientation.w = q.w
                            # odom_data.pose.covariance[35] = self.initial_covariance
                            
                            if len(self.published_headings)>=self.published_headings_length:
                                # filtering out spikes
                                if abs(np.mean(self.published_headings)-(heading+self.heading_offset))<self.heading_diff_publish_ths:
                                    print("PUBLISHING")
                                    self.odom_pub.publish(odom_data)
                                else:
                                    print("===================== NOT PUBLISHING ================================")
                                    print("Heading List: {}".format(self.published_headings))
                                    print("Current Heading: {}".format(heading+self.heading_offset))
                            else:
                                print("++++++++++++++++++++++++ SMALL LENGTH ++++++++++++++++++++++++++++++")
                            self.published_headings.append(heading+self.heading_offset)


                            # self.odom_pub.publish(odom_data)
                        
                else:
                    #TODO: A time-related criterion should clear the deque as well
                    self.last_published_time = rospy.get_time()
                    self.fit_points.clear()

        self.prev_odom = odom_data

    def run(self):
        rospy.spin()

def main():
    gnss_odom = GNSSOdometry()
    gnss_odom.run()
