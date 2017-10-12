#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import copy

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

LOOKAHEAD_WPS = 100  # Number of waypoints we will publish.
BUFFER = 5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', int, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.base_waypoints = None
        self.current_velocity = None
        self.current_pose = None
        self.traffic_waypoint = -2

        self.loop()

    @staticmethod
    def dist(pos_1, pos_2):
        return ((pos_1.x - pos_2.x) ** 2
                + (pos_1.y - pos_2.y) ** 2
                + (pos_1.z - pos_2.z) ** 2) ** 0.5

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if ( (self.base_waypoints is None) or (self.current_velocity is None) or (self.current_pose is None) or
                     (self.traffic_waypoint == -2 ) ):
                continue

            pos = self.current_pose.pose.position

            min_distance = None
            lower_bound = 0
            i = 0
            for bp in self.base_waypoints.waypoints:
                bpos = bp.pose.pose.position
                if bpos.x > pos.x:
                    distance = self.dist(pos, bpos)
                    if (min_distance == None or distance < min_distance):
                        min_distance = distance
                        lower_bound = i
                i += 1

            upper_bound = min(len(self.base_waypoints.waypoints), lower_bound + LOOKAHEAD_WPS)
            lane_waypoints = copy.deepcopy(self.base_waypoints.waypoints[lower_bound:upper_bound])

            if (self.traffic_waypoint != -1 and lower_bound <= self.traffic_waypoint < upper_bound):
                relative_traffic_waypoint = self.traffic_waypoint - lower_bound
                lane_waypoints = self.decelerate_to_index(lane_waypoints, relative_traffic_waypoint)

            lane_msg = Lane()
            lane_msg.waypoints = lane_waypoints

            self.final_waypoints_pub.publish(lane_msg)

            rate.sleep()

    def decelerate_to_index(self, waypoints, redlight_index):
        """
            Changes way point speeds to smoothly come to a stop at the 
            way point of the redlight index.

            Args:
                waypoints: list of waypoints
                redlight_index: int of index of waypoint to stop at

            Returns:
                waypoints: list of waypoints with smoothly decelerating
                speeds.
        """

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

    def traffic_cb(self, traffic_waypoint):
        self.traffic_waypoint = traffic_waypoint.data

    def obstacle_cb(self, msg):
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
