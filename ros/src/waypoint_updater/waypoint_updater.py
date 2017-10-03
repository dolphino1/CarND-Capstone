#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x`
distance ahead.

As mentioned in the doc, you should ideally first implement a version which
does not care about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status
of traffic lights too.

Please note that our simulator also provides the exact location of traffic
lights and their current status in `/vehicle/traffic_lights` message.
You can use this message to build this node as well as to verify your
TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50  # Number of waypoints we will publish.
TARGET_SPEED_MPH = 20
TARGET_SPEED_MPS = (TARGET_SPEED_MPH * 1604.04) / (60 * 60)
BUFFER = 5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', int, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints',
                                                   Lane, queue_size=1)

        self.base_waypoints = None
        self.current_velocity = None
        self.current_pose = None

        rospy.spin()

    @staticmethod
    def dist(pos_1, pos_2):
        return ((pos_1.x - pos_2.x) ** 2
                + (pos_1.y - pos_2.y) ** 2
                + (pos_1.z - pos_2.z) ** 2) ** 0.5

    def update(self, traffic_waypoint_index):
        if not (self.base_waypoints is None or
           self.current_velocity is None or
           self.current_pose is None):

            pos = self.current_pose.pose.position

            min_distance = None
            lower_bound = 0
            i = 0
            for bp in self.base_waypoints.waypoints:
                bpos = bp.pose.pose.position
                if bpos.x > pos.x:
                    bp.twist.twist.linear.x = TARGET_SPEED_MPS
                    distance = self.dist(pos, bpos)
                    if (min_distance is None or distance < min_distance):
                        min_distance = distance
                        lower_bound = i
                i += 1

            upper_bound = min(len(self.base_waypoints.waypoints),
                              lower_bound + LOOKAHEAD_WPS)
            lane_waypoints = self.base_waypoints.waypoints

            traffic_waypoint_index = int(traffic_waypoint_index.data)

            if (traffic_waypoint_index != -1):
                lane_waypoints = self.decelerate_to_index(
                    lane_waypoints, traffic_waypoint_index)

            lane_waypoints = lane_waypoints[lower_bound:upper_bound]

            lane_msg = Lane()
            lane_msg.waypoints = lane_waypoints


        # rospy.logdebug(
        #     "CAR CURRENT pos: (%f, %f, %f), orient: (%f, %f, %f, %f)",
        #     self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z,
        #     self.current_pose.pose.orientation.x, self.current_pose.pose.orientation.y,
        #     self.current_pose.pose.orientation.z, self.current_pose.pose.orientation.w)
        #
        # rospy.logdebug("Lane Waypoints:")
        # count = 0
        # for wp in lane_waypoints:
        #     rospy.logdebug("%d) pos: (%f, %f, %f), orient: (%f, %f, %f, %f), linear: (%f, %f, %f), angular: (%f, %f, %f)",
        #                    count, wp.pose.pose.position.x, wp.pose.pose.position.y, wp.pose.pose.position.z,
        #                    wp.pose.pose.orientation.x, wp.pose.pose.orientation.y, wp.pose.pose.orientation.z,
        #                    wp.pose.pose.orientation.w,
        #                    wp.twist.twist.linear.x, wp.twist.twist.linear.y, wp.twist.twist.linear.z,
        #                    wp.twist.twist.angular.x, wp.twist.twist.angular.y, wp.twist.twist.angular.z)
        #     count += 1

        self.final_waypoints_pub.publish(lane_msg)

    def decelerate_to_index(self, waypoints, redlight_index):

        if len(waypoints) < 1:
            return []

        end = waypoints[redlight_index].pose.pose.position

        for i, wp in enumerate(waypoints):

            if i >= redlight_index:
                velocity = 0
            else:
                dist = self.dist(wp.pose.pose.position, end)
                dist = max(0, dist - BUFFER)
                velocity = dist ** 0.5
                if velocity < 2:
                    velocity = 0.
            wp.twist.twist.linear.x = min(velocity, wp.twist.twist.linear.x)

        return waypoints

    def pose_cb(self, msg):
        self.current_pose = msg

    def waypoints_cb(self, lane):
        self.base_waypoints = lane

    def velocity_cb(self, velocity):
        self.current_velocity = velocity

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.update(msg)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
