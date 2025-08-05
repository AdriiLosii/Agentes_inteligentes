#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
from movementsClass import Movements
import time


# Definimos las variables que se modificaran en el callback del scan junto con la variable de estado
D_FL = 5.0
D_F = 5.0
D_FR = 5.0
STATE = 'wandering'


def scan_cb(data):
    global D_FL
    global D_F
    global D_FR
    global STATE

    # Modificamos la lista para no obtener valores infinitos para valores mayores a 5.0 (rango maximo del escaner laser del seeker)
    ranges = np.array(data.ranges)
    mod_ranges = list(np.where(ranges >= 5.0, 5.0, ranges))

    # Datos necesarios para estados wander y seek:
    D_FL = min(mod_ranges[len(mod_ranges)*4//7:len(mod_ranges)])
    D_F = min(mod_ranges[len(mod_ranges)*3//7:len(mod_ranges)*4//7])
    D_FR = min(mod_ranges[0:len(mod_ranges)*3//7])

    # Datos para estado seek:
    MIN_SCAN = min(mod_ranges)
    NEARBY_STD_D = np.std(mod_ranges[mod_ranges.index(MIN_SCAN)-5:mod_ranges.index(MIN_SCAN)+5])    # Comprobamos los 10 valores alrededor de la distancia minima detectada y calculamos su desviacion estandar

    # Diferenciamos entre robot y pared mediante la desv. estandar calculada anteriormente y un umbral definido experimentalmente
    if (NEARBY_STD_D >= 0.08):  # Es el robot
        STATE = 'seeking'
    else:                       # Es una pared
        STATE = 'wandering'

    # Si la distancia frontal es menor al umbral definido se considerara cazado
    if (MIN_SCAN <= 1.5 and NEARBY_STD_D >= 0.08):
        STATE = 'success'

# Main():
def seeker():
    rospy.init_node('nodo_seeker')
    pub_seeker = rospy.Publisher('/Seeker/cmd_vel', Twist, queue_size=10)
    pub_game = rospy.Publisher('/game', Bool, queue_size=10)
    rospy.Subscriber('/Seeker/laser/scan', LaserScan, scan_cb)
    vel_seeker = Twist()
    rate = rospy.Rate(4)

    while not rospy.is_shutdown():
        # Maquina de estados:
        if (STATE == 'wandering'):
            vel_seeker_0 = Movements().wander(D_FL, D_F, D_FR, 2.0)   # 2.0 es el umbral necesario para considerar que una pared esta demasiado cerca (predefinido: 2.0)
            vel_seeker.linear.x = vel_seeker_0.linear.x*1.25
            vel_seeker.angular.z = vel_seeker_0.angular.z

        if (STATE == 'seeking'):
            vel_seeker = Movements().seek(D_FL, D_FR)

        if (STATE == 'success'):
            vel_seeker = Movements().spin()
            pub_seeker.publish(vel_seeker)
            pub_game.publish(True)  # Publicamos en el topic creado con el hider para indicarle que lo hemos pillado
            break

        pub_seeker.publish(vel_seeker)
        rate.sleep()



if __name__ == '__main__':
    # Iniciamos el temporizador del programa
    inicio = time.time()
    seeker()
    fin = time.time()
    duracion = fin-inicio
    print("Tiempo:",duracion)