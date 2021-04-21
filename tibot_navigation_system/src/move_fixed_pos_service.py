#! /usr/bin/env python

import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from tibot_navigation_system.srv import MoveFixedPosMsg, MoveFixedPosMsgResponse


class MoveFixedPos():

    def __init__(self):
        rospy.init_node('move_fixed_pos_service_node', anonymous=True)
        self.service_server = rospy.Service(
            'move_robot_service', MoveFixedPosMsg, self.srv_callback)
        rospy.loginfo("Service /move_robot_service ready!")

        self.places_dict = dict()
        self.places_dict.update({"bed": (1.0, 1.0, math.sin(self.degree_to_rad(90)), math.cos(self.degree_to_rad(90)))})
        self.places_dict.update({"table": (2.0, 2.0, math.sin(self.degree_to_rad(90)), math.cos(self.degree_to_rad(90)))})
        self.places_dict.update({"wardrobe": (1.5, 2.0, math.sin(self.degree_to_rad(90)), math.cos(self.degree_to_rad(90)))})

        rospy.spin()  # Mantiene el servicio abierto

    def srv_callback(self, request):
        # Obtenemos los datos del mensaje recibido
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
        
        # Creamos una accion cliente llamada "move_base" con una accion MoveBaseAction
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Esperamos hasta que el servidor este listo para escuchar
        client.wait_for_server()

        # Creamos el goal con el constructor MoveBaseGoal
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

        # Envio del goal al action server
        client.send_goal(goal)

        # Esperamos a que el servidor finalize la accion
        wait = client.wait_for_result()

        # Si el resultado no es devuelto, asumimos que el servidor esta offline
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Resultado de la ejecucion de la accion
            response = MoveFixedPosMsgResponse()
            response.success = True
            return response

        return False

    def degree_to_rad(self, deg):
        rad = abs(deg * 3.14 / 180)
        return rad
    

# Main
try:
    MoveFixedPos()
except rospy.ROSInterruptException:
    rospy.loginfo("Navigation has been finished.")