import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class SequenceTrajectory(Node):

    def __init__(self):
        super().__init__('square_trajectory')

        self.pub = self.create_publisher(
            Twist,
            '/drone/cmd_vel',
            10
        )

        # Paramètres
        
        self.turn_speed = 0.5          # rad/s
        
        self.turn_time = (math.pi / 2) / self.turn_speed #Calcul du temps nécessaire pour tourner

        # État au lancement
        

        self.timer = self.create_timer(0.1, self.update)    #timer du node

    def avancer(self, vitesse=0.4, duree=2.0):
        
        msg = Twist()
        msg.linear.x = vitesse

        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds * 1e-9 < duree:
            self.pub.publish(msg)
            time.sleep(0.1)  # publier à 10 Hz

        # Stop le drone après la durée
        stop_msg = Twist()
        self.pub.publish(stop_msg)

    def arret(self, vitesse=0):
        
        msg = Twist()
        msg.linear.x = vitesse
        self.pub.publish(msg)

    def turn_left(self, angle_deg=90, vit_rot=0.2):
        
        msg = Twist()
        #msg.linear.x = 0   #On fait le choix de garder la vitesse et que seul stop arrête le drone

        start_time = self.get_clock().now()
        angle_rad = angle_deg * math.pi / 180
        turn_time = angle_rad / vit_rot
        
        msg.angular.z = vit_rot

        while (self.get_clock().now() - start_time).nanoseconds * 1e-9 < turn_time:
            self.pub.publish(msg)
            time.sleep(0.1)  # publier à 10 Hz

        # Stop le drone après la durée
        stop_msg = Twist()
        self.pub.publish(stop_msg)

    def turn_right(self, angle_deg=90, vit_rot=0.2):
        
        msg = Twist()
        #msg.linear.x = 0   #On fait le choix de garder la vitesse et que seul stop arrête le drone

        start_time = self.get_clock().now()
        angle_rad = angle_deg * math.pi / 180
        turn_time = angle_rad / vit_rot
        
        msg.angular.z = -vit_rot

        while (self.get_clock().now() - start_time).nanoseconds * 1e-9 < turn_time:
            self.pub.publish(msg)
            time.sleep(0.1)  # publier à 10 Hz

        # Stop le drone après la durée
        stop_msg = Twist()
        self.pub.publish(stop_msg)


    def execute_sequence(self, sequence):
        for action_name, params in sequence:
            action_func = getattr(self, action_name, None) # Récupérer la fonction par son nom
            if action_func is not None:
                action_func(**params)  # appelle la fonction avec les paramètres
            else:
                self.get_logger().warn(f"Action inconnue: {action_name}") #Erreur si l'action n'existe pas


def main():
    rclpy.init()
    node = SequenceTrajectory()

    # Séquence à réaliser
    sequence = [
        ('avancer', {'vitesse': 0.4, 'duree': 2.0}),
        ('turn_left', {'angle_deg': 90, 'vit_rot': 0.3}),
        ('avancer', {'vitesse': 0.4, 'duree': 2.0}),
        ('turn_right', {'angle_deg': 90, 'vit_rot': 0.3}),
        ('arret', {})
    ]

    node.execute_sequence(sequence)

    #rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
