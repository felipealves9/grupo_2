#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data
import time
import sys
import math

class SimpleTakeoffLand(Node):
    def __init__(self):
        super().__init__('simple_takeoff_land')
        
        self.current_pose = None
        self.is_scanning = False # Controle de estado
        
        self.pos_sub = self.create_subscription(
            PoseStamped, 
            '/mavros/local_position/pose', 
            self.pose_callback, 
            qos_profile_sensor_data
        )
        
        self.setpoint_pub = self.create_publisher(
            PoseStamped, 
            '/mavros/setpoint_position/local', 
            10
        )

        # Novo publicador para avisar a visão
        self.scan_pub = self.create_publisher(Bool, '/mission/scanning', 10)

        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')

        while rclpy.ok() and not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for MAVROS...')

        self.run_mission()

    def pose_callback(self, msg):
        self.current_pose = msg.pose.position

    def set_mode(self, mode_name):
        req = SetMode.Request()
        req.custom_mode = mode_name
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def takeoff(self, altitude):
        req = CommandTOL.Request()
        req.altitude = altitude
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def land(self):
        req = CommandTOL.Request()
        future = self.land_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def generate_waypoints(self):
        height = 4.5
        length = 14.0
        waypoints = []
        
        waypoints.append([0.0, 0.0, height])
        
        y_lanes = [-4.0, -1.33, 1.33, 4.0]
        x_direction = -length/2
        
        for y in y_lanes:
            waypoints.append([x_direction * -1, y, height])
            waypoints.append([x_direction, y, height])
            x_direction *= -1

        waypoints.append([0.0, 0.0, height])
        
        return waypoints

    def get_distance(self, target_x, target_y, target_z):
        if self.current_pose is None:
            return float('inf')
        
        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y
        dz = target_z - self.current_pose.z
        
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def publish_setpoint(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        self.setpoint_pub.publish(msg)
        
        # Avisa a visão constantemente sobre o status atual
        msg_scan = Bool()
        msg_scan.data = self.is_scanning
        self.scan_pub.publish(msg_scan)

    def run_mission(self):
        try:
            self.get_logger().info('Starting mission...')
            self.set_mode("GUIDED")
            time.sleep(1)
            self.arm()
            time.sleep(1)
            self.takeoff(altitude=4.5)
            time.sleep(8)
            
            route = self.generate_waypoints()
            
            for i, wp in enumerate(route):
                target_x, target_y, target_z = wp
                self.get_logger().info(f'Moving to waypoint {i+1}/{len(route)}: [{target_x}, {target_y}]')
                
                while rclpy.ok():
                    self.publish_setpoint(target_x, target_y, target_z)
                    
                    dist = self.get_distance(target_x, target_y, target_z)
                    if dist < 0.5:
                        # Se chegou no primeiro ponto do zig-zag (canto da arena), libera a visão
                        if i == 1:
                            self.is_scanning = True
                        break
                        
                    rclpy.spin_once(self, timeout_sec=0.1)
            
            self.is_scanning = False # Desliga a visão no pouso
            self.get_logger().info('Scan completed. Landing...')
            self.land()
            time.sleep(8)
            
        except KeyboardInterrupt:
            self.get_logger().warn('Emergency abort triggered!')
            self.land()
            time.sleep(5)
            sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTakeoffLand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
