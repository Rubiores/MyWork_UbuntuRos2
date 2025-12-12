#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import serial
import threading
import math


class PicoBridge(Node):
    def __init__(self):
        super().__init__('pico_bridge')

        # --- parameters (can be overridden via ros2 param)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.0325)        # meters
        self.declare_parameter('wheel_separation', 0.229) #0.239    # meters (track)
        self.declare_parameter('encoder_ticks_per_rev', 10380) # 2325
        self.declare_parameter('odom_publish_rate', 100)

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baudrate').value
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_sep = float(self.get_parameter('wheel_separation').value)
        self.ticks_per_rev = int(self.get_parameter('encoder_ticks_per_rev').value)
        self.odom_rate = float(self.get_parameter('odom_publish_rate').value)

        # tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # serial
        try:
            self.ser = serial.Serial(self.port, int(self.baud), timeout=0.1)
            self.get_logger().info(f"Serial abierto en {self.port} @{self.baud}")
        except Exception as e:
            self.get_logger().error(f"Error abriendo serial: {e}")
            self.ser = None

        # publishers/subscribers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # encoder state (shared with serial thread) --> protect with lock
        self.lock = threading.Lock()
        self.left_ticks = 0
        self.right_ticks = 0

        # joint positions (radians)
        self.left_pos = 0.0
        self.right_pos = 0.0

        # last ticks for odom
        self.last_left_ticks = None
        self.last_right_ticks = None

        # odom pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # last time
        self.last_time = self.get_clock().now()

        # start serial reader thread
        self._serial_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self._serial_thread.start()

        # timer for publishing
        self.create_timer(1.0 / float(self.odom_rate), self._publish_state)

    def cmd_vel_cb(self, msg: Twist):
        # forward / rotate commands are forwarded to the pico controller as-is
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        v_l = v - (w * self.wheel_sep / 2.0)
        v_r = v + (w * self.wheel_sep / 2.0)

        # send to pico as: "CMD <left_m/s> <right_m/s>\n"
        if self.ser:
            line = f"CMD {v_l:.4f} {v_r:.4f}\n"
            try:
                self.ser.write(line.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Error writing serial: {e}")

    def _serial_reader(self):
        # serial lines expected: "ENC <left_ticks> <right_ticks>"
        if not self.ser:
            return
        while rclpy.ok():
            try:
                raw = self.ser.readline()
            except Exception:
                raw = b''
            if not raw:
                continue
            try:
                line = raw.decode('utf-8', errors='ignore').strip()
            except Exception:
                continue
            if not line:
                continue
            parts = line.split()
            if len(parts) >= 3 and parts[0] in ('ENC', 'ENC_D'):
                try:
                    a = int(parts[1])
                    b = int(parts[2])
                except ValueError:
                    continue

                with self.lock:
                    if parts[0] == 'ENC':
                        # backward compatible: absolute counts
                        self.left_ticks = a
                        self.right_ticks = b
                        # update joint positions
                        self.left_pos = 2.0 * math.pi * (self.left_ticks / float(self.ticks_per_rev))
                        self.right_pos = 2.0 * math.pi * (self.right_ticks / float(self.ticks_per_rev))
                    else:
                        # ENC_D -> deltas; sumar al contador interno
                        self.left_ticks += a
                        self.right_ticks += b
                        # update joint positions (integral)
                        self.left_pos += 2.0 * math.pi * (a / float(self.ticks_per_rev))
                        self.right_pos += 2.0 * math.pi * (b / float(self.ticks_per_rev))


    def _compute_tick_delta(self, curr, last):
        # handle potential wrap-around if firmware uses limited counters
        if last is None:
            return 0
        delta = curr - last
        # if counters are unsigned and wrap, attempt simple correction
        # assume wrap if delta is very large in magnitude
        max_ticks = max(1, self.ticks_per_rev)
        if abs(delta) > (max_ticks * 1000):  # heuristic large jump
            # give up, treat as zero delta
            return 0
        return delta

    def _publish_state(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        # copy encoder values atomically
        with self.lock:
            left_ticks = int(self.left_ticks)
            right_ticks = int(self.right_ticks)
            left_pos = float(self.left_pos)
            right_pos = float(self.right_pos)

        # publish joint_states (positions in radians + angular velocity)
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [left_pos, right_pos]

        # compute joint angular velocities from tick deltas
        if self.last_left_ticks is None:
            # first cycle: initialize and publish zeros
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            js.velocity = [0.0, 0.0]
            self.joint_pub.publish(js)
            self.last_time = now
            return

        d_left = self._compute_tick_delta(left_ticks, self.last_left_ticks)
        d_right = self._compute_tick_delta(right_ticks, self.last_right_ticks)

        # update last ticks for next iteration
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks

        # angular displacement (radians) per wheel
        ang_left = (2.0 * math.pi) * (d_left / float(self.ticks_per_rev))
        ang_right = (2.0 * math.pi) * (d_right / float(self.ticks_per_rev))

        w_left = ang_left / dt
        w_right = ang_right / dt
        js.velocity = [w_left, w_right]
        self.joint_pub.publish(js)

        # linear distances travelled by each wheel in meters
        dist_left = ang_left * self.wheel_radius
        dist_right = ang_right * self.wheel_radius

        # linear velocities of wheels
        v_l = dist_left / dt
        v_r = dist_right / dt

        # robot linear and angular velocity
        v = (v_r + v_l) / 2.0
        w = (v_r - v_l) / self.wheel_sep
        
        # --- FILTRO DE ZONA MUERTA (AÑADIR ESTO) ---
        # Si la velocidad es muy pequeña (ruido), la forzamos a cero.
        if abs(v) < 0.05:  # Si es menor a 1 cm/s
            v = 0.0
        if abs(w) < 0.05:  # Si gira muy lento
            w = 0.0
            
        # 3. EL FRENO DE MANO (Anti-Torcedura)
        if v == 0.0 and abs(w) < 0.2:
            w = 0.0


        # integrate pose using current yaw (simple Euler integration)
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += w * dt

        # quaternion from yaw
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        # publish TF odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'  
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # publish odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link' 
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = PicoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
