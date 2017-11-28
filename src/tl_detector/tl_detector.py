#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import numpy as np

import math

STATE_COUNT_THRESHOLD = 2

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=5)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.stop_line_positions = self.config['stop_line_positions']
        self.visible_range = 200 # 300 meter to lookup distance
        self.simulation = False
        self.training = False


        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights



    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def generate_light(self, x, y, z):

        light = TrafficLight()
        light.pose = PoseStamped()
        light.pose.pose.position.x = x
        light.pose.pose.position.y = y
        light.pose.pose.position.z = z
        return light

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        if self.waypoints is not None:
            wpt_list = self.waypoints.waypoints
        else:
            return None
        # Create variables for nearest distance and neighbour
        neighbour_index = None
        neighbour_distance = 9999.0

        # Find Neighbour
        for i in range(len(wpt_list)):
            wpti = wpt_list[i].pose.pose.position
            distance = math.sqrt(
                (wpti.x - pose.position.x) ** 2 + (wpti.y - pose.position.y) ** 2)# + (wpti.z - pose.position.z) ** 2)
            if distance < neighbour_distance:
                neighbour_index = i
                neighbour_distance = distance

        return neighbour_index

    def get_light_state_ground_truth(self, light_index):
        #light_index.state = TrafficLight.GREEN
        return light_index.state

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        stop_line_next_index = None
        light = None
        closest_traffic_light_dist = 99999
        tl_wpt_index = None
        if((self.lights is not None) and (self.waypoints is not None) and (self.pose is not None)):
            # List of positions that correspond to the line to stop in front of for a given intersection
            stop_line_positions = self.config['stop_line_positions']
            if(self.pose):
                car_position = self.get_closest_waypoint(self.pose.pose)

            #TODO find the closest visible traffic light (if one exists)
            stop_lines = list()
            for stop_line in stop_line_positions:
                light =  self.generate_light(stop_line[0], stop_line[1], 0.)
                stop_lines.append(light)

            car_closest_wpt = self.waypoints.waypoints[car_position].pose.pose.position

            for i in range(len(self.lights)):

                traffic_light_pose = self.lights[i].pose.pose
                traffic_light_position = self.get_closest_waypoint(traffic_light_pose)

                tl_wpt_closest = self.waypoints.waypoints[traffic_light_position].pose.pose.position
                distance = math.sqrt( (car_closest_wpt.x - tl_wpt_closest.x) ** 2 +
                                      (car_closest_wpt.y - tl_wpt_closest.y) ** 2 )
                                    #+ (car_closest_wpt.z - tl_wpt_closest.z) ** 2)

                if distance < self.visible_range:
                    if distance < closest_traffic_light_dist:
                        closest_traffic_light_dist = distance
                        tl_wpt_index = i


            #and (car_position < tl_closest_wpt_index_list[tl_wpt_index])
            #if (len(tl_closest_wpt_index_list) != 0) and (len(closest_tl_index_list) != 0):
            if tl_wpt_index is not None:
                 # Check whether the neighbour traffic light is  ahead or behind the car
                car_coordinates = self.pose.pose.position
                car_orientation = self.pose.pose.orientation
                car_quaternion = (car_orientation.x, car_orientation.y, car_orientation.z, car_orientation.w)
                # get cartesian orientation angles
                rotation_pitch_yaw = tf.transformations.euler_from_quaternion(car_quaternion)
                #
                yaw = rotation_pitch_yaw[2]

                # Project a unit vector along orientation
                proj_module = 1.0
                orient_v = np.array([proj_module * math.cos(yaw) ,proj_module * math.sin(yaw)])

                stop_line_next_index = self.get_closest_waypoint(stop_lines[tl_wpt_index].pose.pose)

                stop_line_wpt = self.waypoints.waypoints[stop_line_next_index].pose.pose.position

                diff_v =  np.array([car_coordinates.x - stop_line_wpt.x, car_coordinates.y - stop_line_wpt.y])
                # calculate the vectorial product to determine the orientation
                dot_product = np.dot(orient_v, diff_v)

                x_modulus = np.sqrt((orient_v*orient_v).sum())
                y_modulus = np.sqrt((diff_v*diff_v).sum())
                cos_angle = dot_product / x_modulus / y_modulus # cosine of angle between x and y
                angle = np.arccos(cos_angle) * 360 / 2 / np.pi # angle in degrees

                if abs(angle) < np.pi/4:
                    light = self.lights[tl_wpt_index]



        if light is not None:

            state = self.get_light_state(light)
            light_wp = stop_line_next_index
            #if state == TrafficLight.RED:
                #rospy.loginfo('[TrafficLight] state : %s', state)
                #rospy.loginfo('[TrafficLight] stop_line : %s  distance : %s', light_wp, closest_traffic_light_dist)
            return light_wp, state
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
