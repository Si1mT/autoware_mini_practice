#!/usr/bin/env python3

import math
import rospy

from tf.transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from pyproj import CRS, Transformer, Proj

from novatel_oem7_msgs.msg import INSPVA
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, TransformStamped, Pose, Point

from std_msgs.msg import Header

class Localizer:
    def __init__(self):
        # Parameters
        self.undulation = rospy.get_param('/undulation')
        utm_origin_lat = rospy.get_param('/utm_origin_lat')
        utm_origin_lon = rospy.get_param('/utm_origin_lon')

        # Internal variables
        self.crs_wgs84 = CRS.from_epsg(4326)
        self.crs_utm = CRS.from_epsg(25835)
        self.utm_projection = Proj(self.crs_utm)

        # Subscribers
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.transform_coordinates)

        # Publishers
        self.current_pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=10)
        self.current_velocity_pub = rospy.Publisher('current_velocity', TwistStamped, queue_size=10)
        self.br = TransformBroadcaster()

        # Coordinate transformer
        self.transformer = Transformer.from_crs(self.crs_wgs84, self.crs_utm)
        self.origin_x, self.origin_y = self.transformer.transform(utm_origin_lat, utm_origin_lon)
    
    def transform_coordinates(self, msg):
        # transform received coordinates and subtract the custom origin point near Delta building
        trans_x, trans_y = self.transformer.transform(msg.latitude, msg.longitude)
        trans_x = trans_x - self.origin_x
        trans_y = trans_y - self.origin_y

        # calculate and apply azimuth correction
        azimuth_correction = self.utm_projection.get_factors(msg.longitude, msg.latitude).meridian_convergence
        azimuth = msg.azimuth - azimuth_correction

        # convert corrected azimuth to yaw angle
        yaw = self.convert_azimuth_to_yaw(math.radians(azimuth))

        # Convert yaw to quaternion
        x, y, z, w = quaternion_from_euler(0, 0, yaw)
        orientation = Quaternion(x, y, z, w)

        # take the norm of north direction velocity and east direction velocity
        norm_velocity = math.sqrt(msg.north_velocity**2 + msg.east_velocity**2)

        # current pose message
        current_pose_msg = PoseStamped()
        # header
        current_pose_msg.header.stamp = msg.header.stamp
        current_pose_msg.header.frame_id = "map"
        # pose with orientation
        current_pose_msg.pose.position.x = trans_x
        current_pose_msg.pose.position.y = trans_y
        current_pose_msg.pose.position.z = msg.height - self.undulation
        current_pose_msg.pose.orientation = orientation
        # finally publish the created message
        self.current_pose_pub.publish(current_pose_msg)

        # current velocity message
        current_velocity_msg = TwistStamped()
        # header
        current_velocity_msg.header.stamp = msg.header.stamp
        current_velocity_msg.header.frame_id = "base_link"
        # velocity
        current_velocity_msg.twist.linear.x = norm_velocity
        # publish velocity
        self.current_velocity_pub.publish(current_velocity_msg)

        # transform message
        t = TransformStamped()
        # header
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        # transform
        t.transform.translation = current_pose_msg.pose.position
        t.transform.rotation = current_pose_msg.pose.orientation
        # publish transform
        self.br.sendTransform(t)

        # convert azimuth to yaw angle
    def convert_azimuth_to_yaw(self, azimuth):
        """
        Converts azimuth to yaw. Azimuth is CW angle from the North. Yaw is CCW angle from the East.
        :param azimuth: azimuth in radians
        :return: yaw in radians
        """
        yaw = -azimuth + math.pi/2
        # Clamp within 0 to 2 pi
        if yaw > 2 * math.pi:
            yaw = yaw - 2 * math.pi
        elif yaw < 0:
            yaw += 2 * math.pi

        return yaw

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('localizer')
    node = Localizer()
    node.run()
