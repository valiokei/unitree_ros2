#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math


class SimpleRobotSimulator(Node):
    def __init__(self):
        super().__init__('simple_robot_simulator')
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.z = 0.25  # Altezza del robot
        self.yaw = 0.0
        
        # Subscriber per i comandi di velocità
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/key_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher per le trasformazioni
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer per aggiornare la posizione
        self.timer = self.create_timer(0.01, self.update_position)  # 100 Hz
        
        # Velocità correnti
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        
        self.get_logger().info('Simple Robot Simulator started')
    
    def cmd_vel_callback(self, msg):
        """Callback per i comandi di velocità"""
        self.current_linear_x = msg.linear.x
        self.current_angular_z = msg.angular.z
    
    def update_position(self):
        """Aggiorna la posizione del robot e pubblica la trasformazione"""
        dt = 0.01  # 100 Hz
        
        # Integra le velocità per ottenere la posizione
        if abs(self.current_linear_x) > 0.01 or abs(self.current_angular_z) > 0.01:
            # Movimento lineare nel frame del robot
            dx = self.current_linear_x * dt * math.cos(self.yaw)
            dy = self.current_linear_x * dt * math.sin(self.yaw)
            
            # Aggiorna posizione
            self.x += dx
            self.y += dy
            
            # Aggiorna orientamento
            self.yaw += self.current_angular_z * dt
            
            # Normalizza l'angolo
            while self.yaw > math.pi:
                self.yaw -= 2 * math.pi
            while self.yaw < -math.pi:
                self.yaw += 2 * math.pi
        
        # Pubblica la trasformazione
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        
        # Converti yaw in quaternione
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleRobotSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
