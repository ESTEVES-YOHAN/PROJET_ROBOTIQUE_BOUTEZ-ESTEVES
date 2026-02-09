import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleTrajectory(Node):

    def __init__(self):
        super().__init__('circle_trajectory')

        self.pub = self.create_publisher(
            Twist,
            '/drone/cmd_vel',
            10
        )

        # Timer : appelle self.update toutes les 0.1s
        self.timer = self.create_timer(0.1, self.update)

    def update(self):
        msg = Twist()
        msg.linear.x = 0.4   # vitesse avant
        msg.angular.z = 0.4  # rotation
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CircleTrajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
