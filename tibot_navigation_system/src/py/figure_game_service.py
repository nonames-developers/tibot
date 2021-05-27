#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
This module includes the definition to be able to activate a figure game service
Classes:
    FigureGame
Exceptions:
    ROSInterruptException: Exception for operations that interrupted, e.g. due to shutdown.
Returns:
    FigureGame: A figure game object
Yields:
    []
"""

import rospy
import cv2
from figure import Figure
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tibot_navigation_system.srv import FigureMsg, FigureMsgRequest, FigureMsgResponse


class FigureGame():
    """
    Representation of figure game service

    Attributes:
        service_server (Rospy.Service): Service to listen requests
        bridge_object (CvBridge): To converts between OpenCV images and ROS image messages
        image_sub (Subscriber): Suscribe to robot camera topic
        cv_image (OpenCV): Img msgs to an cv2

    Methods:
        srv_callback(): Callback to check a figure game from robot image capture
        camera_callback(): Callback to receive robot image capture
    """

    def __init__(self):
        rospy.init_node('figure_game_service', anonymous=True)
        self.service_server = rospy.Service(
            'figure_game_robot_service', FigureMsg, self.srv_callback)
        rospy.loginfo("Service /figure_game_robot_service ready!")

        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/turtlebot3/camera/image_raw", Image, self.camera_callback)

        rospy.spin()  # Keep the service open

    def srv_callback(self, request):
        """ Function that receives a str figure and str color and check figure with image

        Args:
            request (FigureMsgRequest): A message that contains a str features of figure (figure, color)

        Returns:
            response (FigureMsgResponse): Success or not
        """
        figure_to_detect = str(request.figure)
        color_to_detect = str(request.color)

        f = Figure(figure_to_detect, color_to_detect)
        state = f.check(self.cv_image)

        while (state == False):
            state = f.check(self.cv_image)
            
        # Result of the execution from the service
        response = FigureMsgResponse()
        if state:
            response.success = True
        else:
            response.success = False

        return response

    def camera_callback(self, data):
        """ Callback to receive robot image capture

        Args:
            data (Img msgs): Data of image capture received
        """

        try:
            self.cv_image = self.bridge_object.imgmsg_to_cv2(
                data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        #print(self.cv_image.shape)
        #print(self.cv_image[400, 400, :]) 
        #print("fotooooooooooooooooooooooo!!!")


def main():
    """ Main function to init FigureGame service

    Exceptions:
        ROSInterruptException: Exception for operations that interrupted, e.g. due to shutdown.
    """
    try:
        FigureGame()
    except rospy.ROSInterruptException:
        rospy.loginfo("Figure game service has been finished.")

####################################################################################################


# Init
main()
