from styx_msgs.msg import TrafficLight
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        # TODO: Surya: please add the yolo calssifier here
        # TODO: teh clasifier will search for traffic lights and then send their bounding boxes if they exists
        # TODO: these bounding boxes cropped image is then evaluated as implemented below in get_classification function
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        ligh_state = TrafficLight.UNKNOWN
        output = image.copy()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0,50,50])
        upper_red = np.array([20,255,255])
        Hue_red_range_1 = cv2.inRange(hsv, lower_red , upper_red)

        lower_red = np.array([171,50,50])
        upper_red = np.array([180,255,255])
        Hue_red_range_2 = cv2.inRange(hsv, lower_red , upper_red)

        #merge both Hue red images
        converted_img = cv2.addWeighted(Hue_red_range_1, 1.0, Hue_red_range_2, 1.0, 0.0)

        blur_img = cv2.GaussianBlur(converted_img,(17,17),0)

        circles = cv2.HoughCircles(blur_img, cv2.HOUGH_GRADIENT, 0.5, 30, param1=35, param2=22, minRadius=4, maxRadius=50)

        if circles is not None:
            ligh_state = TrafficLight.RED

        return ligh_state
