#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from geometry_msgs.msg import TwistStamped

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 70 # Number of waypoints we will publish. You can change this number
MAX_VELOCITY = 40
MAX_DECEL = 1.0
ONE_MPH = 0.44704

class WaypointUpdater(object):
    def __init__(self):

        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.cv_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=4)

        # TODO: Add other member variables you need below
        # car current position
        self.current_pose = None
        # world waypoints
        self.base_waypoints = None
        # traffic light position
        self.traffic_light_wpt = None

        self.ts = None
        self.lowpass_tau = None
        self.current_velocity = None
        self.lowpass_filter = None
        # call the loop_handler
        self.loop_handler()

    def copy_wp(self, waypoint):
        new_wp = Waypoint()
        new_wp.pose.pose.position.x = waypoint.pose.pose.position.x
        new_wp.pose.pose.position.y = waypoint.pose.pose.position.y
        new_wp.pose.pose.position.z = waypoint.pose.pose.position.z
        new_wp.pose.pose.orientation.x = waypoint.pose.pose.orientation.x
        new_wp.pose.pose.orientation.y = waypoint.pose.pose.orientation.y
        new_wp.pose.pose.orientation.z = waypoint.pose.pose.orientation.z
        new_wp.pose.pose.orientation.w = waypoint.pose.pose.orientation.w
        new_wp.twist.twist.linear.x = waypoint.twist.twist.linear.x
        new_wp.twist.twist.linear.y = waypoint.twist.twist.linear.y
        new_wp.twist.twist.linear.z = waypoint.twist.twist.linear.z
        new_wp.twist.twist.angular.x = waypoint.twist.twist.angular.x
        new_wp.twist.twist.angular.y = waypoint.twist.twist.angular.y
        new_wp.twist.twist.angular.z = waypoint.twist.twist.angular.z

        return new_wp

    def cv_cb(self, msg):
        self.current_velocity = msg.twist.linear.x

    def get_nearest_ahead_wpt(self, wpt_list, curr_pose):
        # Create variables for nearest distance and neighbour
        neighbour_index = None
        # Set High value as default
        neighbour_distance_min = 99999
        # Iterate the base_waypoints to find the closest neighbour
        for i in range (len( wpt_list)):
            index = i % len(wpt_list)
            wp_i = wpt_list[index].pose.pose.position
            distance = math.sqrt((wp_i.x - curr_pose.x) ** 2 + (wp_i.y - curr_pose.y) ** 2 + (wp_i.z - curr_pose.z) ** 2 )
            if distance < neighbour_distance_min:
                neighbour_distance_min = distance
                neighbour_index = index

        return neighbour_index

    def decelerate(self, lane_cp, wpt_list, neighbour_index, traffic_light_wpt):
        lane = lane_cp
        lookahead_stop_dist = self.distance(wpt_list, neighbour_index, traffic_light_wpt)
        # find min deceleration distance using Vf**2 = Vi**2 + 2ad
        min_braking_distance = (self.current_velocity**2)/(2*MAX_DECEL)

        #rospy.loginfo('[decelerate]  STOP_LIGNE_wpt    : %s   min_braking_distance    : %s', traffic_light_wpt, min_braking_distance)
        #rospy.loginfo('[decelerate]  neighbour_index   : %s   lookahead_stop_distance : %s', neighbour_index, lookahead_stop_dist)


        #rospy.loginfo('[decelerate] self.current_velocity __________SPEED     : %s', self.current_velocity  )

        if (lookahead_stop_dist < 70) and (min_braking_distance > lookahead_stop_dist - 4):

            min_braking_distance = min(min_braking_distance, lookahead_stop_dist)

            braking_waypoints = neighbour_index
            # Find num of waypoints on the current path that are needed to travel that distance
            for i in range (neighbour_index, LOOKAHEAD_WPS):
                if self.distance (wpt_list, traffic_light_wpt, i) > min_braking_distance:
                    braking_waypoints = i
                    break
            # safety buffer
            braking_clearance = 5
            braking_start_wp = max( 0, braking_waypoints - neighbour_index  - braking_clearance )
            #rospy.loginfo('[decelerate] braking_start_wp    : %s', braking_start_wp)
            if braking_start_wp <= braking_clearance:
                deceleration = 2 * MAX_DECEL
            else:
                deceleration = MAX_DECEL

            #rospy.loginfo('[decelerate] deceleration     : %s', deceleration)
            for i in range ( braking_start_wp, LOOKAHEAD_WPS):
                dec_step = i - braking_start_wp + 1
                lane.waypoints[i].twist.twist.linear.x  = self.current_velocity - (deceleration * dec_step)
                lane.waypoints[i].twist.twist.linear.x  = max(0.00, lane.waypoints[i].twist.twist.linear.x)
                ##rospy.loginfo('[decelerate] SETUP_SPEED     : %s', lane.waypoints[i].twist.twist.linear.x )
        return lane

    def wpt_update(self):

        if ((self.base_waypoints is not None) and (self.current_pose is not None)):
            # Create a Standard Lane Message
            lane = Lane()
            # Set frame header : ID and Timestamp
            lane.header.frame_id = '/neighbour'
            lane.header.stamp = rospy.Time.now()

            # Get current position
            curr_pose = self.current_pose.pose.position
            # Get world waypoints
            wpt_list = self.base_waypoints.waypoints
            # Get TrafficLight position if exists
            traffic_light_wpt = self.traffic_light_wpt
            """
            TODO: add deceleration
            """
            neighbour_index = self.get_nearest_ahead_wpt(wpt_list, curr_pose)
            # Create a lookahead wpts sized list for final waypoints
            # Create a lookahead wps sized list for final waypoints
            for i in range ( neighbour_index, neighbour_index + LOOKAHEAD_WPS):
                # Handle Wraparound
                index = i % len(wpt_list)
                #self.set_waypoint_velocity(wpt_list, index, MAX_VELOCITY * ONE_MPH)
                wpi = self.copy_wp(wpt_list[index])
                lane.waypoints.append(wpi)

            #rospy.loginfo('[wpt_update] SETUP_________SPEED     : %s', lane.waypoints[LOOKAHEAD_WPS- 1].twist.twist.linear.x )
            #rospy.loginfo('[decelerate] self.current_velocity __________SPEED     : %s', self.current_velocity  )

            if (traffic_light_wpt is not None) and (neighbour_index <= traffic_light_wpt) and (self.current_velocity is not None):
                lane = self.decelerate(lane, wpt_list, neighbour_index, traffic_light_wpt)

            # publish the final_waypoints of the closest neighbour
            self.final_waypoints_pub.publish(lane)

    def loop_handler(self):
        # Run the iterations at 10 Hz
        rate = rospy.Rate (5)
        while not rospy.is_shutdown():
            self.wpt_update()
            rate.sleep()

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg
        #pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        #pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        if  msg.data == -1:
            # no red traffic light detected
            self.traffic_light_wpt = None
        else:
            # stop for the ahead traffic light
            #rospy.loginfo("Detected RED light:" + str(msg.data))
            self.traffic_light_wpt = msg.data
        #pass

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
        for i in range(wp1, wp2 + 1):
            index = i % len(waypoints)
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[index].pose.pose.position)
            wp1 = index
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
