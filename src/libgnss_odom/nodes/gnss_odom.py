import rospy
import math

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

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        return q

class GNSSOdometry:
    def __init__(self):
        rospy.init_node('gnss_odom', anonymous=True)
        
        self.use_odometry = rospy.get_param('~use_odometry', False)
        self.velocity_threshold = rospy.get_param('~velocity_threshold', 0.03)
        self.initial_covariance = rospy.get_param('~initial_covariance', 0.1)
        self.orientation_offset = rospy.get_param('~heading_offset',0.0)

        self.prev_fix = None
        self.prev_cmd = None
        self.prev_odom = None

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
        """Stores the last received velocity command."""
        self.prev_cmd = cmd_data

    def compute_odom_from_gnss(self, fix_data):
        """Computes and publishes odometry data from GNSS readings."""
        if self.prev_fix is not None and self.prev_cmd is not None:
            if self.prev_cmd.linear.x >= self.velocity_threshold:
                current_lon = fix_data.longitude
                current_lat = fix_data.latitude
                prev_lon = self.prev_fix.longitude
                prev_lat = self.prev_fix.latitude
                
                X = math.cos(math.radians(current_lat)) * math.sin(math.radians(prev_lon - current_lon))
                Y = math.cos(math.radians(prev_lat)) * math.sin(math.radians(current_lat)) - math.sin(math.radians(prev_lat)) * math.cos(math.radians(current_lat)) * math.cos(math.radians(prev_lon - current_lon))
                heading = math.atan2(X, Y)

                q = Quaternion.from_euler(0.0, 0.0, heading+self.orientation_offset)
                
                current_odom = Odometry()
                current_odom.header.stamp = fix_data.header.stamp
                current_odom.header.frame_id = fix_data.header.frame_id
                current_odom.pose.pose.orientation.x = q.x
                current_odom.pose.pose.orientation.y = q.y
                current_odom.pose.pose.orientation.z = q.z
                current_odom.pose.pose.orientation.w = q.w
                current_odom.pose.covariance[35] = self.initial_covariance
                
                self.odom_pub.publish(current_odom)

        self.prev_fix = fix_data

    def compute_odom_from_odometry(self, odom_data):
        """Computes heading from odometry readings."""
        if self.prev_odom is not None:
            heading = math.atan2(odom_data.pose.pose.position.y - self.prev_odom.pose.pose.position.y,
                                 odom_data.pose.pose.position.x - self.prev_odom.pose.pose.position.x)
            
            q = Quaternion.from_euler(0.0, 0.0, heading+self.orientation_offset)
            
            odom_data.pose.pose.orientation.x = q.x
            odom_data.pose.pose.orientation.y = q.y
            odom_data.pose.pose.orientation.z = q.z
            odom_data.pose.pose.orientation.w = q.w
            odom_data.pose.covariance[35] = self.initial_covariance
            
            self.odom_pub.publish(odom_data)

        self.prev_odom = odom_data

    def run(self):
        rospy.spin()
def main():
    gnss_odom = GNSSOdometry()
    gnss_odom.run()
