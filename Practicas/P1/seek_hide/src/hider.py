#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
from movementsClass import Movements


# Definimos las variables que se modificaran en el callback del scan
D_FL = 2.5
D_F = 2.5
D_FR = 2.5

# Definimos la variable de estado que se modificara en el callback enviado por el seeker
STATE = 'wandering'


def scan_cb(data):
    global D_FL
    global D_F
    global D_FR

    # Modificamos la lista para no obtener valores infinitos para valores mayores a 2.5 (rango maximo del escaner laser del hider)
    ranges = np.array(data.ranges)
    mod_ranges = list(np.where(ranges >= 2.5, 2.5, ranges))

    # Datos necesarios para estado wander:
    D_FL = min(mod_ranges[len(mod_ranges)*5//8:len(mod_ranges)*7//8])
    D_F = min(mod_ranges[len(mod_ranges)*3//8:len(mod_ranges)*5//8])
    D_FR = min(mod_ranges[len(mod_ranges)*1//8:len(mod_ranges)*3//8])

def game_cb(data):  # Este callback viene del seeker, le comunica al hider que ha sido cazado
    global STATE

    if(data):
        STATE = 'seeked'

# Main():
def hider():
    rospy.init_node('nodo_hider')
    pub_hider = rospy.Publisher('/Hider/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/Hider/laser/scan', LaserScan, scan_cb)
    rospy.Subscriber('/game', Bool, game_cb)
    vel_hider = Twist()
    rate = rospy.Rate(4)

    while not rospy.is_shutdown():
        if (STATE == 'wandering'):
            vel_hider = Movements().wander(D_FL, D_F, D_FR, 2.0)    # 2.0 es el umbral necesario para considerar que una pared esta demasiado cerca (predefinido: 2.0)

        if (STATE == 'seeked'):
            vel_hider = Movements().stop()
            pub_hider.publish(vel_hider)
            break

        pub_hider.publish(vel_hider)
        rate.sleep()



if __name__ == '__main__':
    hider()