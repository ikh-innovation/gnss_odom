"""Gets GNSS lat/lon data and publishes heading and speed of robot in UTM frame."""

import serial
import sys

import uncertainties as un
import math

import rospy

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

linear_x_speed_threshold = 0.03

"""
    Quaternion computation with float and uncertanties
"""
class Quaternion:
    w: un.ufloat(0.0, 0.0)
    x: un.ufloat(0.0, 0.0)
    y: un.ufloat(0.0, 0.0)
    z: un.ufloat(0.0, 0.0)

def quaternion_from_euler(roll = un.ufloat(0.0, 0.0), pitch = un.ufloat(0.0, 0.0), yaw = un.ufloat(0.0, 0.0)):
    """
    Converts euler roll, pitch, yaw to quaternion taking into account uncetanties
    """
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

def computeOdom(fix_data, args):
    prev_fix = args[0]
    prev_cmd = args[1]
    odom_pub = args[2]

    # Takes into account previous fix and las cmd_vel stored
    if prev_fix is not None and prev_cmd is not None:

        # To be valid needs a linear speed higher than configured threshold
        if prev_cmd.linear.x >= linear_x_speed_threshold:
            # Create variables with uncertanties
            current_lon = un.ufloat(fix_data.longitude, fix_data.position_covariance[0])
            current_lat = un.ufloat(fix_data.latitude, fix_data.position_covariance[4])

            prev_lon = un.ufloat(prev_fix.longitude, prev_fix.position_covariance[0])
            prev_lat = un.ufloat(prev_fix.latitude, prev_fix.position_covariance[4])

            ## From https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
            # Bearing from point A to B, can be calculated as,
            # β = atan2(X,Y),
            # where, X and Y are two quantities and can be calculated as:
            # X = cos θb * sin ∆L
            # Y = cos θa * sin θb – sin θa * cos θb * cos ∆L

            X = un.cos(current_lat) * un.sin(prev_lon - current_lon)
            Y = un.cos(prev_lat) * un.sin(current_lat) - un.sin(prev_lon) * un.cos(current_lat) * un.cos(prev_lon - current_lon)
            heading = un.atan2(X,Y)

            ## Make use of cmd_vel instead
            ## Speed to filter headings
            # current_stamp = fix_data.heading.stamp.sec + fix_data.heading.stamp.nsec * math.pow(10.0,-9.0)
            # prev_stamp = prev_fix.heading.stamp.sec + prev_fix.heading.stamp.nsec * math.pow(10.0,-9.0)
            # time_diff = current_stamp - prev_stamp

            # distance = un.sqrt(un.pow((current_lon - prev_lon),2) + un.pow((current_lat - prev_lat),2))
            # speed = distance / time_diff # angle / second
            
            ## Odometry message to be published
            current_odom = Odometry()

            current_odom.header.stamp = fix_data.header.stamp
            current_odom.header.frame_id = fix_data.header.frame_id

            q = quaternion_from_euler(un.ufloat(0.0, 0.0), un.ufloat(0.0, 0.0), heading)
            current_odom.pose.pose.orientation.x = q.x.nominal_value
            current_odom.pose.pose.orientation.y = q.y.nominal_value
            current_odom.pose.pose.orientation.z = q.z.nominal_value
            current_odom.pose.pose.orientation.w = q.w.nominal_value

            # Last component of the 6x6 matrix is the rotation around Z axis
            current_odom.pose.covariance[35] = heading.std_dev
            
            odom_pub.publish(current_odom)

    prev_fix = fix_data

def storeCmdVel(cmd_data, args):
    args[0] = cmd_data

def main():
    """
    Create and run the nmea_serial_driver ROS node.

    Create publisher an subscriber for input/output data
    """

    rospy.init_node('gnss_odom')

    prev_fix = None
    prev_cmd = None
    
    cmd_vel_topic = rospy.get_param('~cmd_vel_topic', "husky_velocity_controller/cmd_vel")
    fix_topic = rospy.get_param('~fix_topic', "gnss/fix")
    odom_pub_topic = rospy.get_param('~odom_pub_topic', "gnss/odom")

    odom_pub = rospy.Publisher(odom_pub_topic, Odometry, queue_size=1)
    cmd_vel_sub = rospy.Subscriber(cmd_vel_topic, Twist, storeCmdVel, (prev_cmd))
    fix_sub = rospy.Subscriber(fix_topic, NavSatFix, computeOdom, (prev_fix, prev_cmd, odom_pub))

    rospy.spin()