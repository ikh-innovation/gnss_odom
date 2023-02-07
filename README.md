# gnss_odom

Has a Roslaunch with a rosnode that:
 -Takes consecutive gnss data (NavSatFix) and produces Odom message with heading with its covariance propagate.

Note that this data is only valid when the robot is moving (not point turn, not stopped) so it uses the cmd msg to filter.
