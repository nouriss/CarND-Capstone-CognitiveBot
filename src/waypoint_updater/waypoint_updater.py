#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MAX_VELOCITY = 40
ONE_MPH = 0.44704

class WaypointUpdater(object):
    def __init__(self):

        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

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

    def accelerate(self, lane, wpt_list, neighbour_index):

        for i in range ( neighbour_index, neighbour_index + LOOKAHEAD_WPS ):
            # Handle Wraparound MAX_VELOCITY
            index = i % len(wpt_list)
            filtred_acceleration_velocity = MAX_VELOCITY * ONE_MPH
            self.set_waypoint_velocity(wpt_list, index, filtred_acceleration_velocity)
            wp_i = self.copy_wp(wpt_list[index])
            lane.waypoints.append(wp_i)
        pass

    def decelerate(self, lane, wpt_list, neighbour_index, traffic_light_wpt):

        lookahead_stop_dist = self.distance(wpt_list, neighbour_index, traffic_light_wpt)
        #self.current_pose
        if lookahead_stop_dist > 2:
            set_vel = self.get_waypoint_velocity(wpt_list[neighbour_index])
            steps = abs(traffic_light_wpt - neighbour_index - 3)
            if steps != 0:
                decleratation = set_vel/steps
            else:
                decleratation = set_vel/3

            #correction
            if lookahead_stop_dist > 4 and set_vel < 0.5:
                set_vel = 0.6

            count = 1
            for i in range ( neighbour_index, neighbour_index + traffic_light_wpt + 5):
                # Handle Wraparound
                index = i % len(wpt_list)
                filtred_braking_velocity =  set_vel -  (decleratation    * count)
                count +=1
                if filtred_braking_velocity < 0:
                    filtred_braking_velocity = 0
                if (abs(traffic_light_wpt - index) < 3) or index > traffic_light_wpt:
                    filtred_braking_velocity = 0.0
                if filtred_braking_velocity < 1.0:
                    filtred_braking_velocity = 0.0

                self.set_waypoint_velocity(wpt_list, index, filtred_braking_velocity)
                wp_i = self.copy_wp(wpt_list[index])
                lane.waypoints.append(wp_i)
        else:
            for i in range ( neighbour_index, neighbour_index + traffic_light_wpt + 5):
                # Handle Wraparound
                index = i % len(wpt_list)
                #current_velocity = get_waypoint_velocity(wpt_list[index])
                self.set_waypoint_velocity(wpt_list, index, 0.0)
                wp_i = self.copy_wp(wpt_list[index])
                lane.waypoints.append(wp_i)
            pass

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
            TODO: add deceleration with low passfilter to stop the car at self.traffic_light_wpt
            """
            neighbour_index = self.get_nearest_ahead_wpt(wpt_list, curr_pose)
            # Create a lookahead wpts sized list for final waypoints

            if traffic_light_wpt is not None and  (neighbour_index <= traffic_light_wpt):
                self.decelerate(lane, wpt_list, neighbour_index, traffic_light_wpt)
                #self.accelerate(lane, wpt_list, neighbour_index)
            else:
                self.accelerate(lane, wpt_list, neighbour_index)

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
