#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import json
import time
from datetime import datetime

from std_msgs.msg import String

class MSSHealthMonitorNode(Node):
    """
    Węzeł monitorujący zdrowie systemu MSS.
    Zbiera raporty health od wszystkich węzłów i publikuje ogólny status systemu.
    """
    
    def __init__(self):
        super().__init__('mss_health_monitor_node')
        
        # Parametry
        self.declare_parameter('health_check_interval', 5.0)
        self.declare_parameter('node_timeout_sec', 15.0)
        
        self.health_check_interval = self.get_parameter('health_check_interval').get_parameter_value().double_value
        self.node_timeout_sec = self.get_parameter('node_timeout_sec').get_parameter_value().double_value
        
        # Lista monitorowanych węzłów
        self.monitored_nodes = [
            'gps_rtk_node',
            'bt_receiver_node', 
            'gear_reader_node',
            'servo_controller',
            'gear_shifter',
            'speed_filter_node',
            'speed_controller_node',
            'relative_computer_node',
            'gear_manager_node',
            'diagnostics_node'
        ]
        
        # Stan węzłów
        self.node_states = {}
        self.node_health_data = {}
        self.last_health_updates = {}
        
        # QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishery
        self.system_status_pub = self.create_publisher(String, '/mss/system_status', qos_profile)
        self.node_status_pub = self.create_publisher(String, '/mss/node_status', qos_profile)
        self.health_alerts_pub = self.create_publisher(String, '/mss/health_alerts', qos_profile)
        
        # Subskrypcje na health status węzłów
        for node_name in self.monitored_nodes:
            topic_name = f'/mss/node_health/{node_name}'
            self.create_subscription(
                String, 
                topic_name, 
                lambda msg, name=node_name: self.node_health_callback(msg, name),
                qos_profile
            )
        
        # Timer do publikacji statusu systemu
        self.status_timer = self.create_timer(
            self.health_check_interval, 
            self.publish_system_status
        )
        
        # Inicjalizacja stanu węzłów
        for node_name in self.monitored_nodes:
            self.node_states[node_name] = 'UNKNOWN'
            self.node_health_data[node_name] = {}
            self.last_health_updates[node_name] = 0.0
        
        self.get_logger().info(f"Health Monitor MSS uruchomiony. Monitoruje {len(self.monitored_nodes)} węzłów.")
        self.get_logger().info(f"Interwał sprawdzania: {self.health_check_interval}s")
        self.get_logger().info(f"Timeout węzła: {self.node_timeout_sec}s")
    
    def node_health_callback(self, msg, node_name):
        """Callback dla health status węzłów."""
        try:
            health_data = json.loads(msg.data)
            self.node_states[node_name] = health_data.get('status', 'UNKNOWN')
            self.node_health_data[node_name] = health_data
            self.last_health_updates[node_name] = time.time()
            
            # Logowanie błędów i ostrzeżeń
            errors = health_data.get('errors', [])
            warnings = health_data.get('warnings', [])
            
            if errors:
                self.get_logger().warn(f"Węzeł {node_name} zgłosił błędy: {errors}")
            if warnings:
                self.get_logger().info(f"Węzeł {node_name} zgłosił ostrzeżenia: {warnings}")
                
        except json.JSONDecodeError:
            self.get_logger().error(f"Nieprawidłowy format JSON od węzła {node_name}")
            self.node_states[node_name] = 'ERROR'
    
    def check_node_timeouts(self):
        """Sprawdza timeout węzłów na podstawie ostatnich raportów health."""
        current_time = time.time()
        
        for node_name in self.monitored_nodes:
            time_since_update = current_time - self.last_health_updates[node_name]
            
            # Sprawdzanie timeout
            if time_since_update > self.node_timeout_sec:
                if self.node_states[node_name] != 'TIMEOUT':
                    self.get_logger().warn(f"Węzeł {node_name} - timeout ({time_since_update:.1f}s)")
                    self.node_states[node_name] = 'TIMEOUT'
            elif self.node_states[node_name] == 'TIMEOUT':
                # Reset timeout status jeśli węzeł znowu raportuje
                self.node_states[node_name] = 'UNKNOWN'
    
    def publish_system_status(self):
        """Publikuje ogólny status systemu."""
        # Sprawdź timeout węzłów
        self.check_node_timeouts()
        
        # Oblicz ogólny status
        total_nodes = len(self.monitored_nodes)
        running_nodes = sum(1 for state in self.node_states.values() if state == 'running')
        error_nodes = sum(1 for state in self.node_states.values() if state in ['ERROR', 'TIMEOUT', 'FAILED'])

        if error_nodes == 0:
            overall_status = 'OK'
        elif error_nodes < total_nodes // 2:
            overall_status = 'WARNING'
        else:
            overall_status = 'ERROR'
        
        system_status = {
            'timestamp': datetime.now().isoformat(),
            'overall_status': overall_status,
            'total_nodes': total_nodes,
            'running_nodes': running_nodes,
            'error_nodes': error_nodes,
            'node_states': self.node_states,
            'node_health_data': self.node_health_data
        }
        
        status_msg = String()
        status_msg.data = json.dumps(system_status)
        self.system_status_pub.publish(status_msg)
        
        # Publikacja alertów jeśli są problemy
        if error_nodes > 0:
            alert_msg = String()
            alert_data = {
                'timestamp': datetime.now().isoformat(),
                'level': 'WARNING' if overall_status == 'WARNING' else 'ERROR',
                'message': f"System MSS: {error_nodes} węzłów ma problemy",
                'details': {name: state for name, state in self.node_states.items() if state != 'running'}
            }
            alert_msg.data = json.dumps(alert_data)
            self.health_alerts_pub.publish(alert_msg)
        
        # Publikacja szczegółowego statusu węzłów
        node_status_msg = String()
        node_status_data = {
            'timestamp': datetime.now().isoformat(),
            'nodes': {}
        }
        
        for node_name in self.monitored_nodes:
            node_status_data['nodes'][node_name] = {
                'status': self.node_states[node_name],
                'last_update': self.last_health_updates[node_name],
                'health_data': self.node_health_data[node_name]
            }
        
        node_status_msg.data = json.dumps(node_status_data)
        self.node_status_pub.publish(node_status_msg)

def main(args=None):
    rclpy.init(args=args)
    health_monitor_node = MSSHealthMonitorNode()
    
    try:
        rclpy.spin(health_monitor_node)
    except KeyboardInterrupt:
        health_monitor_node.get_logger().info("Health Monitor MSS zatrzymywany...")
    finally:
        health_monitor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
