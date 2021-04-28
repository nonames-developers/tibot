#!/usr/bin/env python

import rospy
import smach
import time
from smach import State, StateMachine
from smach_ros import IntrospectionServer

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from tf.transformations import quaternion_from_euler
from collections import OrderedDict


# Puntos de escondite
waypoints = [
    ['GO_TO_BED', (-0.3, 2.5, 0.0), (0.0, 0.0, 1.0, 0.0)],
    ['GO_TO_TABLE', (1.2, 2.4, 0.0), (0.0, 0.0, 1.0, 0.0)],
    ['GO_TO_WARDROBE', (2.1, 3.0, 0.0), (0.0, 0.0, 1.0, 0.0)]
]

# Descripcion de los estados

# Encender Robot


class PowerOnRobot(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo("Encendiendo el Robot")
        time.sleep(2)
        return 'succeeded'

# Robot esperando orden


class WaitingOrder(State):
    def __init__(self, order_state):
        State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=[
                       ''], output_keys=[''])
        self.order = order_state

    def execute(self, userdata):
        if self.order == 1:
            return 'succeeded'
        else:
            return 'aborted'

# Robot se mueve a un sitio


class Navigate(State):
    def __init__(self, position, orientation, place):
        State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=[
                       ''], output_keys=[''])
        self._position = position
        self._orientation = orientation
        self._place = place
        self._move_base = actionlib.SimpleActionClient(
            "/move_base", MoveBaseAction)
        rospy.loginfo("Activando el cliente de navegacion..")
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
        # Comprobamos el estado de la navegacion
        nav_state = self._move_base.get_state()
        rospy.loginfo("[Result] State: %d" % (nav_state))
        nav_state = 3

        if nav_state == 3:
            return 'succeeded'
        else:
            return 'aborted'

# Buscando al Robot


class FindRobot(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        print("Buscando al robot...")
        for cont in range(0, 5):
            cont += 1
            time.sleep(1)
            print(".")
        if cont > 0:
            return 'succeeded'
        else:
            return 'aborted'

# Cargar Robot


class Charge(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=[
                       'input'], output_keys=[''])

    def execute(self, userdata):
        print("Revisando la carga de la bateria...")
        if userdata.input == 1:
            print("Robot cargado")
            return 'succeeded'
        else:
            print("Robot sin carga")
            return 'aborted'


class main():
    def __init__(self):
        rospy.init_node('move_base_action_client', anonymous=False)
        sm_esconder = StateMachine(outcomes=['succeeded', 'aborted'])
        sm_esconder.userdata.sm_input = 1

        with sm_esconder:
            StateMachine.add('POWER_ON', PowerOnRobot(), transitions={
                             'succeeded': 'WAITING_ORDER', 'aborted': 'aborted'})
            StateMachine.add('WAITING_ORDER', WaitingOrder(1), transitions={
                             'succeeded': waypoints[2][0], 'aborted': 'WAITING_ORDER'})
            StateMachine.add(waypoints[2][0], Navigate(waypoints[2][1], waypoints[2][2], waypoints[2][0]), transitions={
                             'succeeded': 'FINDING_ROBOT', 'aborted': 'WAITING_ORDER'})
            StateMachine.add('FINDING_ROBOT', FindRobot(), transitions={
                             'succeeded': waypoints[5][0], 'aborted': 'WAITING_ORDER'})
            StateMachine.add(waypoints[3][0], Navigate(waypoints[3][1], waypoints[3][2], waypoints[3][0]), transitions={
                             'succeeded': 'CHARGE', 'aborted': 'aborted'})
            StateMachine.add('CHARGE', Charge(), transitions={
                             'succeeded': 'succeeded', 'aborted': 'aborted'}, remapping={'input': 'sm_input', 'output': ''})
        intro_server = IntrospectionServer('Tibot', sm_esconder, '/SM_ROOT')
        intro_server.start()

        # Ejecutamos la maquina de estados
        sm_outcome = sm_esconder.execute()
        intro_server.stop()

    def shutdown(self):
        rospy.loginf("Parando la ejecucion...")
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Testeo Tibot finalizado")
