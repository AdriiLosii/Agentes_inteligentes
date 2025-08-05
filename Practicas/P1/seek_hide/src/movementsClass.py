#!/usr/bin/env python3
from geometry_msgs.msg import Twist


class Movements:
    def __init__(self):
        self.vel = Twist()

    def wander(self, D_FL, D_F, D_FR, THRESHOLD = 2.0):
        """
        Metodo para que el robot deambule por el mundo evitando obstaculos
        """

        # Condicion de avance:
        if (D_F > THRESHOLD): # 2.0 es el umbral necesario para considerar que una pared esta demasiado cerca
            self.vel.linear.x = 0.8
        else:
            self.vel.linear.x = 0.0

        # Condicion de giro:
        if (D_FL > D_FR):     # Gira izquierda
            self.vel.angular.z = -0.6
        elif (D_FL < D_FR):   # Gira derecha
            self.vel.angular.z = 0.6
        else:
            self.vel.angular.z = 0.0

        return self.vel

    def seek(self, D_FL, D_FR):
        """
        Metodo para que el robot persiga al objeto mas proximo
        """

        # Condicion de avance:
        self.vel.linear.x = 1.0

        # Condicion de giro:
        if (D_FL > D_FR):
            self.vel.angular.z = 0.8
        else:
            self.vel.angular.z = -0.8

        return self.vel

    def spin(self):
        """
        Metodo para que el robot gire sobre si mismo
        """

        self.vel.linear.x = 0.0
        self.vel.angular.z = 1.0

        return self.vel

    def stop(self):
        """
        Metodo para que el robot se detenga por completo
        """

        self.vel.linear.x = 0
        self.vel.angular.z = 0

        return self.vel