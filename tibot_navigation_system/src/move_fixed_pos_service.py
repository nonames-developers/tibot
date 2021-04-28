#! /usr/bin/env python

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

import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from tibot_navigation_system.srv import MoveFixedPosMsg, MoveFixedPosMsgRequest, MoveFixedPosMsgResponse


class MoveFixedPos():
    """
    Representation of Tibot movement

    Attributes:
        service_server (Rospy.Service): Service to listen requests
        places_dict (Dict): Dict object

    Methods:
        srv_callback(): Callback to move to received position 
        degree_to_rad(): Convert degrees to radians
    """

    def __init__(self):
        rospy.init_node('move_fixed_pos_service_node', anonymous=True)
        self.service_server = rospy.Service(
            'move_robot_service', MoveFixedPosMsg, self.srv_callback)
        rospy.loginfo("Service /move_robot_service ready!")

        self.places_dict = dict()
        self.places_dict.update(
            {"bed": (-0.5, 2.5, math.sin(self.degree_to_rad(90)), math.cos(self.degree_to_rad(90)))})
        self.places_dict.update({"table": (1.2, 2.4, math.sin(
            self.degree_to_rad(180)), math.cos(self.degree_to_rad(180)))})
        self.places_dict.update({"wardrobe": (2.1, 3.0, math.sin(
            self.degree_to_rad(180)), math.cos(self.degree_to_rad(180)))})

        rospy.spin()  # Keep the service open

    def srv_callback(self, request):
        """ Function that receives a place and performs a robot movement 

        Args:
            request (MoveFixedPosMsgRequest): A message that contains a place to move

        Returns:
            [type]: [description]
        """
        # We obtain the data of the received message
        place = str(request.place).lower()
        if self.places_dict.get(place) is not None:
            rospy.loginfo("The selected place is %s", place)
            x_pos = self.places_dict.get(place)[0]
            rospy.loginfo("The x position is %s", x_pos)
            y_pos = self.places_dict.get(place)[1]
            rospy.loginfo("The y position is %s", y_pos)
            z_or = self.places_dict.get(place)[2]
            rospy.loginfo("The z orientation is %s", z_or)
            w_or = self.places_dict.get(place)[3]
            rospy.loginfo("The w orientation is %s", w_or)

            # We create a client action called "move_base" with a MoveBaseAction action
            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

            # We wait until the server is ready to listen
            client.wait_for_server()

            # We create the goal with the MoveBaseGoal constructor
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            # x position
            goal.target_pose.pose.position.x = x_pos
            # y position
            goal.target_pose.pose.position.y = y_pos
            # z orientation
            goal.target_pose.pose.orientation.z = z_or
            # w orientation
            goal.target_pose.pose.orientation.w = w_or

            # Sending the goal to the action server
            client.send_goal(goal)

            # We wait for the server to finish the action
            wait = client.wait_for_result()

            # If the result is not returned, we assume that the server is offline
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                # Result of the execution of the action
                response = MoveFixedPosMsgResponse()
                response.success = True
                return response

        return False

    def degree_to_rad(self, deg):
        """ Convert degrees to radians

        Args:
            deg (int): Degrees

        Returns:
            rad (float): Radians
        """
        rad = abs(deg * math.pi / 180)
        return rad


def main():
    """ Main function to init MoveFixedPos

    Raises:
        ROSInterruptException: Exception for operations that interrupted, e.g. due to shutdown.
    """
    try:
        MoveFixedPos()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation has been finished.")

####################################################################################################


# Init
main()
