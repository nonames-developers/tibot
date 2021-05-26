#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
This module includes the definition to be able to activate a service for the robot movements 
Classes:
    MoveFixedPos
Exceptions:
    ROSInterruptException: Exception for operations that interrupted, e.g. due to shutdown.
Returns:
    MoveFixedPos: A movement object
Yields:
    []
"""

import numpy
import rospy
import cv2
from figure import Figure
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tibot_navigation_system.srv import FigureMsg, FigureMsgRequest, FigureMsgResponse


class FigureGame():
    """
    Representation of Tibot movement
    Attributes:
        service_server (Rospy.Service): Service to listen requests
        places_dict (Dict): Dict object
    Methods:
        srv_callback(): Callback to move to received position 
        degree_to_rad(): Convert degrees to radians
    """

    # 1: MENSAJE DE LAS FIGURAS rquest y response (START Y SUCCESS)
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
        """ Function that receives a place and performs a robot movement 
        Args:
            request (MoveFixedPosMsgRequest): A message that contains a place to move
        Returns:
            [type]: [description]
        """
        figure_to_detect = str(request.figure)
        color_to_detect = str(request.color)

        f = Figure(figure_to_detect, color_to_detect)
        state = f.check(self.cv_image)

        # Result of the execution from the service
        response = FigureMsgResponse()
        if state:
            response.success = True
        else:
            response.success = False

        return response

    def camera_callback(self, data):
        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            self.cv_image = self.bridge_object.imgmsg_to_cv2(
                data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        print("fotooooooooooooooooooooooo")
        cv2.imshow("holaaaa", self.cv_image)
        print(self.cv_image.shape)
        print(self.cv_image[400, 400, :])
        rospy.sleep(5)


def main():
    """ Main function to init MoveFixedPos
    Exceptions:
        ROSInterruptException: Exception for operations that interrupted, e.g. due to shutdown.
    """
    try:
        FigureGame()
    except rospy.ROSInterruptException:
        rospy.loginfo("Figure service has been finished.")

####################################################################################################


# Init
main()
