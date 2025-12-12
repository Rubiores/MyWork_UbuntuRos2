#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # ---- Parámetros del robot ----
        self.declare_parameter('wheel_radius', 0.032)   # metros
        self.declare_parameter('wheel_base', 0.18)      # distancia entre ruedas

        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_base').value

        # ---- Timer ----
        self.last_cmd_vel_time = self.get_clock().now()

        # ---- Suscripción ----
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # ---- Conexión Serial ----
        try:
            self.ser = serial.Serial(
                '/dev/ttyACM0', 115200, timeout=0.02
            )
            self.get_logger().info("Conectado a /dev/ttyACM0")
        except Exception as e:
            self.get_logger().error(f"Error abriendo serial: {e}")
            self.ser = None

        # ---- Loop a 20 ms ----
        self.timer = self.create_timer(0.02, self.control_loop)

        # Velocidades
        self.v_left = 0.0
        self.v_right = 0.0

    def cmd_vel_callback(self, msg):
        # Guardar tiempo
        self.last_cmd_vel_time = self.get_clock().now()

        # Velocidad lineal y angular
        v = msg.linear.x
        w = msg.angular.z

        # Cinemática diferencial
        self.v_left  = v - (w * self.L / 2.0)
        self.v_right = v + (w * self.L / 2.0)

    def control_loop(self):
        # Timeout de 0.5 s ? detener motores
        if (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds > 5e8:
            self.v_left = 0.0
            self.v_right = 0.0

        # Mandar por Serial
        if self.ser:
            try:
                msg = f"{self.v_left:.3f} {self.v_right:.3f}\n"
                self.get_logger().info(f"TX: {msg.strip()}")
                self.ser.write(msg.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Error enviando serial: {e}")

def main():
    rclpy.init()
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
