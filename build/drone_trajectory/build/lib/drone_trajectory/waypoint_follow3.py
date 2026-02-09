import rclpy
from rclpy.node import Node

import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Range


def calculate_angle(x_quad, y_quad, x_target, y_target, theta):
    dx = x_target - x_quad
    dy = y_target - y_quad

    angle_to_target = np.arctan2(dy, dx)
    angle_error = angle_to_target - theta

    # Normalisation pour prendre le chemin court
    angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi

    return angle_error


def calc_dist(x_quad, y_quad, x_target, y_target):
    dist = np.sqrt((x_quad - x_target) ** 2 + (y_quad - y_target) ** 2)
    return dist


class WaypointFollower(Node):

    def __init__(self):
        super().__init__("waypoint_follower")

        self.distance = None
        self.angle_error = None
        self.max_hauteur = 8
        self.wait_timer = 0

        self.Kp_linear = 0.25
        self.Kp_angle = 0.25

        self.tol_dist = 1.0
        self.tol_angle = 0.1

        self.target_pos = [
            (0, 0),
            (-3.45, -9.0),
        ]

        self.target_reached = 0

        self.state = "NAVIGATION"

        self.x = None
        self.y = None
        self.z = None

        self.qx = None
        self.qy = None
        self.qz = None
        self.qw = None

        # Subscription à la position du drone
        self.subscriber_ = self.create_subscription(
            Pose,
            '/drone/gt_pose',
            self.pose_callback,
            10
        )
        self.get_logger().info("Position récupérée !")

        # Subscription à la position du drone
        self.subscriber_ = self.create_subscription(
            Range,
            '/drone/sonar',
            self.sonar_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.copter_ctrl)

        # Publisher des commandes de vitesse
        self.pub_spd = self.create_publisher(
            Twist,
            '/drone/cmd_vel',
            10
        )

    def sonar_callback(self, msg):

        self.hauteur = msg.range

    def pose_callback(self, msg):

        # Position
        self.x = msg.position.x
        self.y = msg.position.y
        self.z = msg.position.z

        # Orientation quaternion
        self.qx = msg.orientation.x
        self.qy = msg.orientation.y
        self.qz = msg.orientation.z
        self.qw = msg.orientation.w

        r = R.from_quat([self.qx, self.qy, self.qz, self.qw])
        self.roll, self.pitch, self.yaw = r.as_euler('xyz', degrees=False)

    def copter_ctrl(self):
        
        if None in (self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw):
            return

        msg_spd = Twist()
        
        self.distance = calc_dist(
            self.x,
            self.y,
            self.target_pos[self.target_reached][0],
            self.target_pos[self.target_reached][1]
        )

        self.angle_error = calculate_angle(
            self.x,
            self.y,
            self.target_pos[self.target_reached][0],
            self.target_pos[self.target_reached][1],
            self.yaw
        )

        speed = self.Kp_linear * self.distance
        turn_speed = self.Kp_angle * self.angle_error

        if self.state == "NAVIGATION":
            if abs(self.angle_error) >= self.tol_angle:
                msg_spd.linear.x = 0.0
                msg_spd.angular.z = turn_speed
            elif abs(self.angle_error) <= self.tol_angle:
                msg_spd.angular.z = 0.0
                msg_spd.linear.x = speed

                if self.distance <= self.tol_dist :
                    self.target_reached += 1
                    if self.target_reached == len(self.target_pos):
                        self.target_reached = 0
                        self.state = "UP"
        
        elif self.state == "UP":
            self.get_logger().info("Montée à 8m")
            msg_spd.linear.z = 1.0
            if self.hauteur >= self.max_hauteur :
                self.state = "DOWN"

        elif self.state == "DOWN":
            msg_spd.linear.z = -5.0
            if self.hauteur <= 3.0 :
                self.state = "RECOVERY"

        elif self.state == "RECOVERY":
            msg_spd.linear.z = 2.0
            if self.hauteur >= 3.0 :
                self.get_logger().info("Sequence terminée")
                self.state = "NAVIGATION"
            
        
        self.pub_spd.publish(msg_spd)



def main(args=None):
    rclpy.init(args=args)

    node = WaypointFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
