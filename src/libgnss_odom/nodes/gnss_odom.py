import rospy
import uncertainties as un
import math

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Quaternion:
    def __init__(self, w=0.0, x=0.0, y=0.0, z=0.0):
        self.w = un.ufloat(w, 0.0)
        self.x = un.ufloat(x, 0.0)
        self.y = un.ufloat(y, 0.0)
        self.z = un.ufloat(z, 0.0)
    @staticmethod
    def from_euler(roll=un.ufloat(0.0, 0.0), pitch=un.ufloat(0.0, 0.0), yaw=un.ufloat(0.0, 0.0)):
        cy = un.cos(yaw * 0.5)
        sy = un.sin(yaw * 0.5)
        cp = un.cos(pitch * 0.5)
        sp = un.sin(pitch * 0.5)
        cr = un.cos(roll * 0.5)
        sr = un.sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        return q
    

class GNSSOdometry:
    def __init__(self):
        rospy.init_node('gnss_odom', anonymous=True)
        
        self.velocity_threshold = rospy.get_param('~velocity_threshold', 0.03)
        self.initial_covariance = rospy.get_param('~initial_covariance', 0.1)

        self.prev_fix = None
        self.prev_cmd = None

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
        
        rospy.Subscriber(
            rospy.get_param('~fix_topic', 'gnss/fix'),
            NavSatFix,
            self.compute_odom
        )

    def store_cmd_vel(self, cmd_data):
        """Stores the last received velocity command."""
        self.prev_cmd = cmd_data

    def compute_odom(self, fix_data):
        """Computes and publishes odometry data from GNSS readings."""
        if self.prev_fix is not None and self.prev_cmd is not None:
            if self.prev_cmd.linear.x >= self.velocity_threshold:
                # Create variables with uncertainties
                current_lon = un.ufloat(fix_data.longitude, self.initial_covariance)
                current_lat = un.ufloat(fix_data.latitude, self.initial_covariance)
                prev_lon = un.ufloat(self.prev_fix.longitude, self.initial_covariance)
                prev_lat = un.ufloat(self.prev_fix.latitude, self.initial_covariance)
                
                # Compute heading angle
                X = un.cos(current_lat) * un.sin(prev_lon - current_lon)
                Y = un.cos(prev_lat) * un.sin(current_lat) - un.sin(prev_lat) * un.cos(current_lat) * un.cos(prev_lon - current_lon)
                heading = un.atan2(X, Y)

                # Convert heading to quaternion
                q = quaternion_from_euler(un.ufloat(0.0, 0.0), un.ufloat(0.0, 0.0), heading)
                
                # Publish Odometry message
                current_odom = Odometry()
                current_odom.header.stamp = fix_data.header.stamp
                current_odom.header.frame_id = fix_data.header.frame_id
                current_odom.pose.pose.orientation.x = q.x.nominal_value
                current_odom.pose.pose.orientation.y = q.y.nominal_value
                current_odom.pose.pose.orientation.z = q.z.nominal_value
                current_odom.pose.pose.orientation.w = q.w.nominal_value
                current_odom.pose.covariance[35] = heading.std_dev
                
                self.odom_pub.publish(current_odom)

        self.prev_fix = fix_data

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    gnss_odom = GNSSOdometry()
    gnss_odom.run()
