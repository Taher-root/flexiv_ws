import rclpy
from rclpy.node import Node
from flexiv_amr_msgs.msg import AMRStatus, BlockedStatus, EmergencyStatus

# Continuously polls the Flexiv AMR for status information and publishes it to ROS topics. Acts as a "health monitor" that other nodes can subscribe to.

class StatusMonitor(Node):
    def __init__(self):
        super().__init__('status_monitor')
        
        self.declare_parameter('amr_ip', '192.168.1.110')
        self.declare_parameter('status_rate', 2.0)
        
        self.amr_ip = self.get_parameter('amr_ip').value
        self.status_rate = self.get_parameter('status_rate').value
        
        self.status_pub = self.create_publisher(AMRStatus, '/amr/status', 10)
        self.blocked_pub = self.create_publisher(BlockedStatus, '/amr/blocked', 10)
        self.emergency_pub = self.create_publisher(EmergencyStatus, '/amr/emergency', 10)
        
        self.states_api = None
        
        self.timer = self.create_timer(1.0 / self.status_rate, self.publish_status)
        
        self.connect_to_amr()
    
    def connect_to_amr(self):
        try:
            import flexivamr
            self.states_api = flexivamr.StatesAPI()
            self.states_api.connect(self.amr_ip)
            self.get_logger().info(f'Status monitor connected to {self.amr_ip}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect status monitor: {e}')
            self.states_api = None

    def publish_status(self):
        if self.states_api is None:
            return
        
        try:
            battery = self.states_api.check_battery_status()
            emergency = self.states_api.check_emergency_status()
            blocked = self.states_api.check_blocked_status()
            control_seized = self.states_api.is_control_seized()
            
            status_msg = AMRStatus()
            status_msg.header.stamp = self.get_clock().now().to_msg()
            status_msg.header.frame_id = 'base_link'
            
            # Battery level (0-100)
            status_msg.battery_percentage = float(battery.battery_level)
            
            # Emergency status
            status_msg.emergency_stop = bool(emergency.emergency)
            
            # Blocked status
            status_msg.blocked = bool(blocked.blocked)
            status_msg.block_reason = str(blocked.block_reason) if blocked.blocked else ''
            
            # Control status
            status_msg.control_seized = bool(control_seized)
            status_msg.control_holder = ''
            
            self.status_pub.publish(status_msg)
            
            # Publish blocked status if blocked
            if status_msg.blocked:
                blocked_msg = BlockedStatus()
                blocked_msg.header.stamp = status_msg.header.stamp
                blocked_msg.header.frame_id = 'base_link'
                blocked_msg.blocked = True
                blocked_msg.reason = status_msg.block_reason
                blocked_msg.slowed = bool(blocked.slowed)
                self.blocked_pub.publish(blocked_msg)
            
            # Publish emergency status if emergency
            if status_msg.emergency_stop:
                emergency_msg = EmergencyStatus()
                emergency_msg.header.stamp = status_msg.header.stamp
                emergency_msg.header.frame_id = 'base_link'
                emergency_msg.emergency_button_pressed = bool(emergency.emergency)
                emergency_msg.driver_emergency = bool(emergency.driver_emc)
                emergency_msg.relay_on = bool(emergency.electric)
                self.emergency_pub.publish(emergency_msg)
                
        except Exception as e:
            self.get_logger().error(f'Failed to get AMR states: {e}')

    # def publish_status(self):
    #     if self.states_api is None:
    #         return
        
    #     try:
    #         # Get status objects
    #         battery = self.states_api.check_battery_status()
    #         emergency = self.states_api.check_emergency_status()
    #         blocked = self.states_api.check_blocked_status()
    #         control_seized = self.states_api.is_control_seized()
            
    #         # Publish AMR status
    #         status_msg = AMRStatus()
    #         status_msg.header.stamp = self.get_clock().now().to_msg()
    #         status_msg.header.frame_id = 'base_link'
            
    #         # Battery percentage (check if attribute exists)
    #         if hasattr(battery, 'percentage'):
    #             status_msg.battery_percentage = float(battery.percentage)
    #         elif hasattr(battery, 'battery_percentage'):
    #             status_msg.battery_percentage = float(battery.battery_percentage)
    #         else:
    #             status_msg.battery_percentage = 0.0
            
    #         # Emergency status
    #         if hasattr(emergency, 'emergency_stop'):
    #             status_msg.emergency_stop = bool(emergency.emergency_stop)
    #         else:
    #             status_msg.emergency_stop = False
            
    #         # Blocked status
    #         if hasattr(blocked, 'blocked'):
    #             status_msg.blocked = bool(blocked.blocked)
    #             if status_msg.blocked and hasattr(blocked, 'reason'):
    #                 status_msg.block_reason = str(blocked.reason)
    #             else:
    #                 status_msg.block_reason = ''
    #         else:
    #             status_msg.blocked = False
    #             status_msg.block_reason = ''
            
    #         # Control status
    #         status_msg.control_seized = bool(control_seized)
    #         status_msg.control_holder = ''  # API doesn't expose this
            
    #         self.status_pub.publish(status_msg)
            
    #         # Publish blocked status if blocked
    #         if status_msg.blocked:
    #             blocked_msg = BlockedStatus()
    #             blocked_msg.header.stamp = status_msg.header.stamp
    #             blocked_msg.header.frame_id = 'base_link'
    #             blocked_msg.blocked = True
    #             blocked_msg.reason = status_msg.block_reason
    #             blocked_msg.slowed = False  # API doesn't expose this
    #             self.blocked_pub.publish(blocked_msg)
            
    #         # Publish emergency status if emergency
    #         if status_msg.emergency_stop:
    #             emergency_msg = EmergencyStatus()
    #             emergency_msg.header.stamp = status_msg.header.stamp
    #             emergency_msg.header.frame_id = 'base_link'
    #             emergency_msg.emergency_button_pressed = True
    #             emergency_msg.driver_emergency = False  # API doesn't expose details
    #             emergency_msg.relay_on = False  # API doesn't expose this
    #             self.emergency_pub.publish(emergency_msg)
                
    #     except Exception as e:
    #         self.get_logger().error(f'Failed to get AMR states: {e}')
    
    
def main(args=None):
    rclpy.init(args=args)
    node = StatusMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()