#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import threading
from .robokit_protocol import (
    API_PORT_CTRL,
    robot_control_motion_req,
    packMsg,
    create_velocity_command,
    create_stop_command
)

class RobokitVelocityController(Node):
    def __init__(self):
        super().__init__('robokit_velocity_controller')
        
        self.declare_parameter('amr_ip', '192.168.1.110')
        self.declare_parameter('ctrl_port', API_PORT_CTRL)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('socket_timeout', 0.1)
        
        self.amr_ip = self.get_parameter('amr_ip').value
        self.ctrl_port = self.get_parameter('ctrl_port').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.socket_timeout = self.get_parameter('socket_timeout').value
        
        self.sock = None
        self.sock_lock = threading.Lock()
        self.serial_num = 1
        
        self.current_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.actual_vel_pub = self.create_publisher(Twist, '/amr/actual_velocity', 10)
        
        self.connect_to_robot()
        
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Robokit velocity controller initialized')
    
    def connect_to_robot(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.amr_ip, self.ctrl_port))
            self.sock.settimeout(self.socket_timeout)
            self.get_logger().info(f'Connected to Robokit CTRL at {self.amr_ip}:{self.ctrl_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            self.sock = None
    
    def cmd_vel_callback(self, msg):
        self.current_cmd.linear.x = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        self.current_cmd.linear.y = max(-self.max_linear, min(self.max_linear, msg.linear.y))
        self.current_cmd.angular.z = max(-self.max_angular, min(self.max_angular, msg.angular.z))
        self.last_cmd_time = self.get_clock().now()
    
    def control_loop(self):
        if self.sock is None:
            return
        
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        vx = self.current_cmd.linear.x
        vy = self.current_cmd.linear.y
        wz = self.current_cmd.angular.z
        
        is_zero_cmd = abs(vx) < 0.001 and abs(vy) < 0.001 and abs(wz) < 0.001
        
        if is_zero_cmd or time_since_cmd > self.cmd_timeout:
            cmd = create_stop_command()
        else:
            cmd = create_velocity_command(vx, vy, wz)
        
        try:
            with self.sock_lock:
                msg = packMsg(self.serial_num, robot_control_motion_req, cmd)
                self.sock.send(msg)
                self.serial_num += 1
                
                try:
                    data = self.sock.recv(16)
                except socket.timeout:
                    pass
            
            actual_vel = Twist()
            actual_vel.linear.x = vx
            actual_vel.linear.y = vy
            actual_vel.angular.z = wz
            self.actual_vel_pub.publish(actual_vel)
            
        except Exception as e:
            self.get_logger().error(f'Control command failed: {e}')
    
    def __del__(self):
        if self.sock:
            try:
                msg = packMsg(self.serial_num, robot_control_motion_req, create_stop_command())
                self.sock.send(msg)
                self.sock.close()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = RobokitVelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
