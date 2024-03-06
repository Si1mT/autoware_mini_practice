#!/usr/bin/python3
import rospy

from autoware_msgs.msg import Lane, VehicleCmd
from geometry_msgs.msg import PoseStamped

from shapely.geometry import LineString, Point
from shapely import prepare, distance

from tf.transformations import euler_from_quaternion
import numpy as np
import math

class PurePursuitFollower:
    def __init__(self):

        # Parameters
        self.path_linestring = LineString()
        self.lookahead_distance = rospy.get_param("~lookahead_distance")
        self.wheel_base = rospy.get_param("/wheel_base")

        # Publishers
        self.vehicle_cmd_pub = rospy.Publisher('/control/vehicle_cmd', VehicleCmd)

        # Subscribers
        rospy.Subscriber('path', Lane, self.path_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

    def path_callback(self, msg):
        # convert waypoints to shapely linestring
        path_linestring = LineString([(w.pose.pose.position.x, w.pose.pose.position.y) for w in msg.waypoints])
        # prepare path - creates spatial tree, making the spatial queries more efficient
        prepare(path_linestring)
        self.path_linestring = path_linestring


    def current_pose_callback(self, msg):
        # checks if path has been created, otherwise return
        if self.path_linestring == None: return

        # local path linestring
        path_linestring = self.path_linestring

        # convert ego vehicle location to shapely Point
        current_pose = Point([msg.pose.position.x, msg.pose.position.y])
        # find the distance using project function
        d_ego_from_path_start = self.path_linestring.project(current_pose)

        # print('d_ego_from_path_start:', d_ego_from_path_start)

        # using euler_from_quaternion to get the heading angle
        _, _, heading = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

        # find lookahead point
        lookahead_point = path_linestring.interpolate(self.lookahead_distance)

        # lookahead point heading calculation
        lookahead_heading = np.arctan2(lookahead_point.y - current_pose.y, lookahead_point.x - current_pose.x)

        # lookahead distance
        lookahead_heading_distance = distance(current_pose, lookahead_point)

        # difference between car heading and lookahead heading
        heading_diff = math.radians(np.abs(heading - lookahead_heading)) # radians?

        # steering angel calculation
        steering_angle = np.arctan((2*self.wheel_base * np.sin(heading_diff)) / lookahead_heading_distance)

        vehicle_cmd = VehicleCmd()
        vehicle_cmd.header.stamp = msg.header.stamp
        vehicle_cmd.header.frame_id = "base_link"
        vehicle_cmd.ctrl_cmd.steering_angle = steering_angle
        vehicle_cmd.ctrl_cmd.linear_velocity = 10.0

        self.vehicle_cmd_pub.publish(vehicle_cmd)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_follower')
    node = PurePursuitFollower()
    node.run()
