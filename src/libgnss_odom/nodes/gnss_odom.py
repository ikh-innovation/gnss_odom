import rospy
import math
import numpy as np
from pyproj import Geod
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

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
        self.distance_threshold = rospy.get_param('~distance_threshold', 0.5)
        self.initial_covariance = rospy.get_param('~initial_covariance', 0.1)
        self.heading_offset = rospy.get_param('~heading_offset',0.0)
        self.gnss_ellipsoid = rospy.get_param('~gnss_ellipsoid','WGS84')
        self.prev_fix = None
        self.prev_cmd = None
        self.prev_odom = None
        
        # Line fitting feature
        self.use_fitted_heading = rospy.get_param('~use_fitted_heading', False)
        self.num_fit_points = rospy.get_param('~num_fit_points', 5)
        if (self.num_fit_points<3):
            self.num_fit_points = 2
            rospy.logwarn("Default number of fit points set: {}".format())
        
        self.fit_points = [] 
        
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
        """Compute heading using line fitting and determine the correct direction."""
        if len(self.fit_points) < 2:
            self.fit_points = []
            return None, self.initial_covariance

        # Extract x and y coordinates
        x_vals = np.array([p[0] for p in self.fit_points])
        y_vals = np.array([p[1] for p in self.fit_points])

        # ----- Least Squares Fitting -----
        A = np.vstack([x_vals, np.ones(len(x_vals))]).T
        m, b = np.linalg.lstsq(A, y_vals, rcond=None)[0]  # Solve y = mx + b
        heading = math.atan(m)  # Convert slope to angle

        # ----- Compute Direction Correction -----
        x_first, y_first = self.fit_points[0]
        x_last, y_last = self.fit_points[-1]
        self.fit_points = []
        
        dx = x_last - x_first
        dy = y_last - y_first

        
        # Compute the direction angle (atan2 gives correct quadrant)
        direction_angle = math.atan2(dy, dx)

        # Determine if the heading needs to be flipped
        # if self.prev_cmd.linear.x < 0:  # Robot moving backward
        #     heading += math.pi  # Flip direction by 180 degrees

        # Keep heading within [-pi, pi]
        # heading = (heading + math.pi) % (2 * math.pi) - math.pi

        # Compute deviation-based covariance
        y_predicted = m * x_vals + b
        residuals = y_vals - y_predicted
        covariance = np.var(residuals) if len(residuals) > 1 else self.initial_covariance

        
        # print("Fitted Heading: {}".format(math.degrees(heading)))
        # print("Direction Angle (atan2): {}".format(math.degrees(direction_angle)))
        # print("Velocity: {} (Flipped: {})".format(self.prev_cmd.linear.x, self.prev_cmd.linear.x < 0))
        # print("Residual covariance: {}".format(covariance))
        
        return heading,covariance


    def compute_odom_from_gnss(self, fix_data):
        print("Fit point list len: {}".format(len(self.fit_points)))
        if self.prev_fix is not None and self.prev_cmd is not None:
            if self.prev_cmd.linear.x >= self.velocity_threshold:
                heading_, _, distance = self.geod.inv( self.prev_fix.longitude, self.prev_fix.latitude, fix_data.longitude, fix_data.latitude)
                heading = None
                if distance >= self.distance_threshold:

                    if (self.use_fitted_heading):
                        x, y, _ = self.geod.fwd(
                        self.prev_fix.longitude, self.prev_fix.latitude,
                        fix_data.longitude, fix_data.latitude
                        )
                        self.fit_points.append((x, y))
                        if len(self.fit_points)>self.num_fit_points:
                            self.fit_points.pop(0)
                            heading, covariance = self.compute_fitted_heading()
                            print("TAKIS")
                    else:
                        heading = math.radians(heading_)
                        covariance = self.initial_covariance

                    print("PRINT")
                    if (heading != None):
                        print("Heading: {} | Cov: {}".format(heading,covariance))
                        q = Quaternion.from_euler(0.0, 0.0, heading+self.heading_offset)
                        
                        current_odom = Odometry()
                        current_odom.header.stamp = fix_data.header.stamp
                        current_odom.header.frame_id = fix_data.header.frame_id
                        current_odom.pose.pose.orientation.x = q.x
                        current_odom.pose.pose.orientation.y = q.y
                        current_odom.pose.pose.orientation.z = q.z
                        current_odom.pose.pose.orientation.w = q.w
                        current_odom.pose.covariance[35] = covariance
                        
                        self.odom_pub.publish(current_odom)
                else:
                    self.fit_points = []

        self.prev_fix = fix_data

    def compute_odom_from_odometry(self, odom_data):
        print("Fit point list len: {}".format(len(self.fit_points)))
        if self.prev_odom is not None and self.prev_cmd is not None:
            
            if self.prev_cmd.linear.x >= self.velocity_threshold:
                
                # Compute distance moved
                dx = odom_data.pose.pose.position.x - self.prev_odom.pose.pose.position.x
                dy = odom_data.pose.pose.position.y - self.prev_odom.pose.pose.position.y
                distance = math.sqrt(dx**2 + dy**2)
                heading = None
                if (distance >= self.distance_threshold):
                    
                    if (self.use_fitted_heading):
                        self.fit_points.append((odom_data.pose.pose.position.x, odom_data.pose.pose.position.y))
                        if len(self.fit_points) > self.num_fit_points:
                            heading, covariance = self.compute_fitted_heading()

                    else:
                        heading = math.atan2(dy, dx)
                        covariance = self.initial_covariance
                    
                    if heading!=None :
                        
                        print("Heading: {} | Cov: {}".format(heading,covariance))
                        q = Quaternion.from_euler(0.0, 0.0, heading+self.heading_offset)
                        
                        odom_data.pose.pose.orientation.x = q.x
                        odom_data.pose.pose.orientation.y = q.y
                        odom_data.pose.pose.orientation.z = q.z
                        odom_data.pose.pose.orientation.w = q.w
                        # odom_data.pose.covariance[35] = self.initial_covariance
                        
                        self.odom_pub.publish(odom_data)
                else:
                    self.fit_points = []

        self.prev_odom = odom_data

    def run(self):
        rospy.spin()

def main():
    gnss_odom = GNSSOdometry()
    gnss_odom.run()
