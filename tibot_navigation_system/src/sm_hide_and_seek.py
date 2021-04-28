#!/usr/bin/env python

"""
IS NOT USED BECAUSE IS NOT NECESARY TO MOVE TO FIXED POSITIONS - MAYBE WE USE IT IN FUTURE INTEGRATIONS 

This module includes the definition to be able to activate a state machine for the robot movements in hide and seek game

Classes:
    HideAndSeekSM

Exceptions:
    ROSInterruptException: Exception for operations that interrupted, e.g. due to shutdown.

Returns:
    HideAndSeekSM: A state machine instance

Yields:
    []
"""

import rospy
import smach
import time
from smach import State, StateMachine
from smach_ros import IntrospectionServer

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from tf.transformations import quaternion_from_euler
from collections import OrderedDict


class PowerOn(State):
    """
    Representation of PowerOn state

    Attributes:
        []

    Methods:
        execute(): Callback to execute actions
    """

    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo("Starting Tibot")
        time.sleep(2)
        return 'succeeded'


class WaitingOrder(State):
    """
    Representation of WaitingOrder state

    Attributes:
        []

    Methods:
        execute(): Callback to execute actions
    """
    def __init__(self, order_state):
        State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=[
                       ''], output_keys=[''])
        self.order = order_state

    def execute(self, userdata):
        if self.order == 1:
            return 'succeeded'
        else:
            return 'aborted'


class Navigate(State):
    """
    Representation of Navigate state

    Attributes:
        []

    Methods:
        execute(): Callback to execute actions
    """
    def __init__(self, position, orientation, place):
        State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=[
                       ''], output_keys=[''])
        self._position = position
        self._orientation = orientation
        self._place = place
        self._move_base = actionlib.SimpleActionClient(
            "/move_base", MoveBaseAction)
        rospy.loginfo("Starting navigate client...")
        self._move_base.wait_for_server(rospy.Duration(15))

    def execute(self, userdata):
        time.sleep(2)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        rospy.loginfo(self._position)
        goal.target_pose.pose.position.x = self._position[0]
        goal.target_pose.pose.position.y = self._position[1]
        goal.target_pose.pose.position.z = self._position[2]
        goal.target_pose.pose.orientation.x = self._orientation[0]
        goal.target_pose.pose.orientation.y = self._orientation[1]
        goal.target_pose.pose.orientation.z = self._orientation[2]
        goal.target_pose.pose.orientation.w = self._orientation[3]

        rospy.loginfo("ROBOT %s" % (self._place))
        # sends the goal
        self._move_base.send_goal(goal)
        self._move_base.wait_for_result()
        # Check navigation state
        nav_state = self._move_base.get_state()
        rospy.loginfo("[Result] State: %d" % (nav_state))
        nav_state = 3

        if nav_state == 3:
            return 'succeeded'
        else:
            return 'aborted'


class Hidden(State):
    """
    Representation of Hidden state

    Attributes:
        []

    Methods:
        execute(): Callback to execute actions
    """
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        print("Looking for the robot...")
        for cont in range(0, 5):
            cont += 1
            time.sleep(1)
            print(".")
        if cont > 0:
            return 'succeeded'
        else:
            return 'aborted'


class Charging(State):
    """
    Representation of Charging state

    Attributes:
        []

    Methods:
        execute(): Callback to execute actions
    """
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=[
                       'input'], output_keys=[''])

    def execute(self, userdata):
        print("Checking batery charge...")
        if userdata.input == 1:
            print("Robot charged")
            return 'succeeded'
        else:
            print("Robot without charge")
            return 'aborted'


class HideAndSeekSM():
    """ State Machine class

    Attributes:
        sm (StateMachine): State machine for navigate
        intro_server (IntrospectionServer): state display

    Methods:
        []

    Exceptions:
        ROSInterruptException: Exception for operations that interrupted, e.g. due to shutdown.
    """

    waypoints = [
        ['GO_TO_BED', (-0.3, 2.5, 0.0), (0.0, 0.0, 1.0, 0.0)],
        ['GO_TO_TABLE', (1.2, 2.4, 0.0), (0.0, 0.0, 1.0, 0.0)],
        ['GO_TO_WARDROBE', (2.1, 3.0, 0.0), (0.0, 0.0, 1.0, 0.0)]]

    def __init__(self):
        rospy.init_node('sm_hide_and_seek', anonymous=False)
        self.sm = StateMachine(outcomes=['succeeded', 'aborted'])
        self.sm.userdata.sm_input = 1

        with self.sm:
            StateMachine.add('POWER_ON', PowerOn(), transitions={
                             'succeeded': 'WAITING_ORDER', 'aborted': 'aborted'})
            StateMachine.add('WAITING_ORDER', WaitingOrder(1), transitions={
                             'succeeded': waypoints[2][0], 'aborted': 'WAITING_ORDER'})
            StateMachine.add(waypoints[2][0], Navigate(waypoints[2][1], waypoints[2][2], waypoints[2][0]), transitions={
                             'succeeded': 'HIDDEN', 'aborted': 'WAITING_ORDER'})
            StateMachine.add('HIDDEN', Hidden(), transitions={
                             'succeeded': waypoints[5][0], 'aborted': 'WAITING_ORDER'})
            StateMachine.add(waypoints[3][0], Navigate(waypoints[3][1], waypoints[3][2], waypoints[3][0]), transitions={
                             'succeeded': 'CHARGING', 'aborted': 'aborted'})
            StateMachine.add('CHARGING', Charging(), transitions={
                             'succeeded': 'succeeded', 'aborted': 'aborted'}, remapping={'input': 'sm_input', 'output': ''})
        self.intro_server = IntrospectionServer('Tibot', self.sm, '/SM_ROOT')
        self.intro_server.start()

        # Execute state machine
        self.sm.execute()
        intro_server.stop()

####################################################################################################


# Init
try:
    HideAndSeekSM()
except rospy.ROSInterruptException:
    rospy.loginfo("State machine has been closed!")
