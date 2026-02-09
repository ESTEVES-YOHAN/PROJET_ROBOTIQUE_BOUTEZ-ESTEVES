import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class SquareTrajectory(Node):

    def __init__(self):
        super().__init__('square_trajectory')

        self.pub = self.create_publisher(
            Twist,
            '/drone/cmd_vel',
            10
        )

        # Paramètres
        self.forward_speed = 0.4       # m/s
        self.turn_speed = 0.5          # rad/s
        self.side_length = 2.0         # m

        self.forward_time = self.side_length / self.forward_speed # Calcul du temps nécessaire pour avancer d'une longueur spécifié
        self.turn_time = (math.pi / 2) / self.turn_speed #Calcul du temps nécessaire pour tourner

        # État au lancement
        self.state = 'FORWARD'
        self.state_start = self.get_clock().now()   #Temps au départ de l'action

        self.timer = self.create_timer(0.1, self.update)    #timer du node

    def update(self):   #Va update l'état et donc les vitesses
        msg = Twist()
        now = self.get_clock().now()
        elapsed = (now - self.state_start).nanoseconds * 1e-9

        if self.state == 'FORWARD':
            msg.linear.x = self.forward_speed

            if elapsed >= self.forward_time: #Si temps nécessaire à parcourir la distance passer, alors changer d'état
                self.state = 'TURN'
                self.state_start = now

        elif self.state == 'TURN':  
            msg.angular.z = self.turn_speed

            if elapsed >= self.turn_time:   #Si tourner fini alors avancer
                self.state = 'FORWARD'
                self.state_start = now

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SquareTrajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
