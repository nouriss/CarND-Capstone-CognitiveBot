#!/usr/bin/env python

import rospy
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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # car current position
        self.current_pose = None
        # world waypoints 
        self.base_waypoints = None
        
        # call the loop_handler 
        #rospy.spin()
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
    
    def wpt_update(self):
        
        if ((self.base_waypoints is not None) and (self.current_pose is not None)):
            # Create a Standard Lane Message
            lane = Lane()
            # Set frame header : ID and Timestamp
            lane.header.frame_id = '/neighbour'
            lane.header.stamp = rospy.Time.now ()

            # Get current position
            curr_pose = self.current_pose.pose.position
            # Get world waypoints 
            wpt_list = self.base_waypoints.waypoints

            # Create variables for nearest distance and neighbour
            neighbour_index = None
            # Set High value as default
            neighbour_distance_min = 99999

            # Iterate the base_waypoints to find the closest neighbour
            for i in range (len( wpt_list)):
                wp_i = wpt_list[i].pose.pose.position
                distance = math.sqrt((wp_i.x - curr_pose.x) ** 2 + (wp_i.y - curr_pose.y) ** 2 + (wp_i.z - curr_pose.z) ** 2 )
                if distance < neighbour_distance_min:
                    neighbour_distance_min = distance
                    neighbour_index = i

            # Create a lookahead wpts sized list for final waypoints
            for i in range ( neighbour_index, neighbour_index + LOOKAHEAD_WPS ):
                # Handle Wraparound
                index = i % len(wpt_list)
                wp_i = self.copy_wp(wpt_list[index])
                lane.waypoints.append(wp_i)
                
            # publish the final_waypoints of the closest neighbour
            self.final_waypoints_pub.publish ( lane )
    
    def loop_handler(self):
        # Run the iterations at 10 Hz
        rate = rospy.Rate (10)
        while not rospy.is_shutdown():
            self.wpt_update()
            rate.sleep()
        
    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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
