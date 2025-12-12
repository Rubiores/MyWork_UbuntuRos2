#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import tf_transformations

class SimpleNav(Node):
    def __init__(self):
        super().__init__('simple_nav')

        self.goal = None
        self.current_pose = None

        # subscripciones
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

        # publicador /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # loop 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose

    def goal_cb(self, msg):
        self.goal = msg.pose
        self.get_logger().info("Nuevo goal recibido")

    def control_loop(self):
        if self.goal is None or self.current_pose is None:
            return

        # posición actual
        x = self.current_pose.position.x
        y = self.current_pose.position.y

        # objetivo
        gx = self.goal.position.x
        gy = self.goal.position.y

        dx = gx - x
        dy = gy - y
        dist = math.sqrt(dx*dx + dy*dy)

        # si llegó, parar
        if dist < 0.10:
            self.cmd_pub.publish(Twist())
            return

        # orientación actual
        q = self.current_pose.orientation
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )

        # yaw deseado
        target_yaw = math.atan2(dy, dx)
        yaw_error = math.atan2(math.sin(target_yaw - yaw),
                               math.cos(target_yaw - yaw))

        cmd = Twist()

        
        # 1. Control angular (P)
        
        K_ang = 1.8
        cmd.angular.z = K_ang * yaw_error

        # saturación del giro
        cmd.angular.z = max(min(cmd.angular.z, 1.5), -1.5)

       
        # 2. Control lineal según alineación
       
        max_angle_for_moving = math.radians(20)  # 20°
        if abs(yaw_error) < max_angle_for_moving:
            # avanza proporcional a la distancia
            K_lin = 0.6
            cmd.linear.x = max(0.15, min(K_lin * dist, 0.35))
        else:
            # si está chueco, quedarse quieto mientras gira
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
