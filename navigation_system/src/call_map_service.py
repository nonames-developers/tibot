#! /usr/bin/env python

import rospy
from nav_msgs.srv import GetMap, GetMapRequest

rospy.init_node('map_client') # se inicializa el nodo cliente
rospy.wait_for_service('/static_map') # esperamos a que el servicio este activo
get_map_service = rospy.ServiceProxy('/static_map', GetMap)  # se crea la conexion al servicio
get_map = GetMapRequest() # se crea un objeto del tipo GetMapRequest
result = get_map_service(get_map) # se llama al servicio y se obtiene el resultado
print(result) # se imprime el resultado
