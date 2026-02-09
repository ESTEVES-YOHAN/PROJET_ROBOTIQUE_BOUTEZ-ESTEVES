import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist #Type de donnée utilisé
import math
import time

class SquareTrajectoryNode(Node):
    def __init__(self):
        super().__init__("trajectory_node")
        #Code of my node here

        self.publisher_ = self.create_publisher(Twist,'/drone/cmd_vel',10) #On va publier des messages Twist sur /drone/cmd_vel avec une liste d'attente de 10
        self.start_time = self.get_clock().now() #On enregistre le temps de démarrage du node
        self.timer = self.create_timer(0.1, self.timer_callback) #On définit le nombre de fois où le node va s'allumer

    def timer_callback(self):
            msg = Twist()
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

            if elapsed < 30:
                #msg.linear.x = 0.3
                msg.angular.z = 1.0
            else:
                msg.linear.x = 0.0

            self.publisher_.publish(msg)


def main():
    rclpy.init()
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown() 

if __name__ == "__main__":
    main()