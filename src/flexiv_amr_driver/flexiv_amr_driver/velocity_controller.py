import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from flexiv_amr_msgs.srv import GainControl, ReleaseControl
import math

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        
        self.declare_parameter('amr_ip', '192.168.1.110')
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('rotation_threshold', 0.05)
        self.declare_parameter('auto_gain_control', True)
        self.declare_parameter('cmd_timeout', 0.5)
        
        self.amr_ip = self.get_parameter('amr_ip').value
        self.control_rate = self.get_parameter('control_rate').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.rotation_threshold = self.get_parameter('rotation_threshold').value
        self.auto_gain_control = self.get_parameter('auto_gain_control').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.gain_control_srv = self.create_service(
            GainControl, 'gain_control', self.gain_control_callback)
        self.release_control_srv = self.create_service(
            ReleaseControl, 'release_control', self.release_control_callback)
        
        self.actual_vel_pub = self.create_publisher(Twist, '/amr/actual_velocity', 10)
        
        self.current_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.control_seized = False
        self.is_moving = False
        
        self.navigator_api = None
        self.configure_api = None
        
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
        
        self.connect_to_amr()
        
        if self.auto_gain_control:
            self.seize_control()
    
    def connect_to_amr(self):
        try:
            import flexivamr
            self.navigator_api = flexivamr.NavigatorAPI(verbose=False)
            self.configure_api = flexivamr.ConfigureAPI(verbose=False)
            
            self.navigator_api.connect(self.amr_ip)
            self.configure_api.connect(self.amr_ip)
            
            self.get_logger().info(f'Connected to AMR at {self.amr_ip}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to AMR: {e}')
            self.navigator_api = None
            self.configure_api = None
    
    def seize_control(self):
        if self.configure_api is None:
            self.get_logger().warn('Cannot seize control - not connected')
            return False
        
        try:
            self.configure_api.gain_control('ros2_nav')
            self.control_seized = True
            self.get_logger().info('Control seized successfully')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to seize control: {e}')
            return False
    
    def release_control_internal(self):
        if self.configure_api is None:
            return False
        
        try:
            self.configure_api.release_control()
            self.control_seized = False
            self.get_logger().info('Control released')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to release control: {e}')
            return False
    
    def cmd_vel_callback(self, msg):
        self.current_cmd.linear.x = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        self.current_cmd.linear.y = max(-self.max_linear, min(self.max_linear, msg.linear.y))
        self.current_cmd.angular.z = max(-self.max_angular, min(self.max_angular, msg.angular.z))
        self.last_cmd_time = self.get_clock().now()
    
    def control_loop(self):
        if not self.control_seized or self.navigator_api is None:
            return
        
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        vx = self.current_cmd.linear.x
        vy = self.current_cmd.linear.y
        wz = self.current_cmd.angular.z
        
        is_zero_cmd = abs(vx) < 0.001 and abs(vy) < 0.001 and abs(wz) < 0.001
        
        if is_zero_cmd or time_since_cmd > self.cmd_timeout:
            if self.is_moving:
                try:
                    self.navigator_api.cancel_current_navigation()
                    self.is_moving = False
                    self.get_logger().debug('Stopped robot')
                except Exception as e:
                    self.get_logger().error(f'Failed to stop: {e}')
            return
        
        dt = 1.0 / self.control_rate
        
        try:
            linear_speed = math.sqrt(vx**2 + vy**2)
            
            if linear_speed < self.rotation_threshold and abs(wz) > 0.01:
                angle = wz * dt
                self.navigator_api.rotation(angle, abs(wz), 0)
                self.is_moving = True
                self.get_logger().debug(f'Rotating: angle={angle:.3f}, speed={abs(wz):.3f}')
            
            elif linear_speed > 0.01:
                dist = linear_speed * dt
                self.navigator_api.translation(dist, vx, vy, 0)
                self.is_moving = True
                self.get_logger().debug(f'Moving: dist={dist:.3f}, vx={vx:.3f}, vy={vy:.3f}')
                
                if abs(wz) > 0.01:
                    angle = wz * dt
                    self.navigator_api.rotation(angle, abs(wz), 0)
                    self.get_logger().debug(f'+ Rotating: angle={angle:.3f}')
            
        except Exception as e:
            self.get_logger().error(f'Control command failed: {e}')
        
        actual_vel = Twist()
        actual_vel.linear.x = vx
        actual_vel.linear.y = vy
        actual_vel.angular.z = wz
        self.actual_vel_pub.publish(actual_vel)
    
    def gain_control_callback(self, request, response):
        if self.seize_control():
            response.success = True
            response.message = 'Control seized successfully'
        else:
            response.success = False
            response.message = 'Failed to seize control'
        return response
    
    def release_control_callback(self, request, response):
        if self.release_control_internal():
            response.success = True
            response.message = 'Control released successfully'
        else:
            response.success = False
            response.message = 'Failed to release control'
        return response
    
    def __del__(self):
        if self.is_moving and self.navigator_api:
            try:
                self.navigator_api.cancel_current_navigation()
            except:
                pass
        
        if self.control_seized:
            self.release_control_internal()

def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
