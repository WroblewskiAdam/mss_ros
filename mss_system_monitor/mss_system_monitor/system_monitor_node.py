#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import psutil
import subprocess
import os
from datetime import datetime

class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__('system_monitor_node')
        
        # Parametry
        self.declare_parameter('monitor_interval', 5.0)
        self.declare_parameter('temperature_warning_threshold', 70.0)
        self.declare_parameter('cpu_warning_threshold', 80.0)
        self.declare_parameter('memory_warning_threshold', 85.0)
        self.declare_parameter('disk_warning_threshold', 90.0)
        
        # Pobierz parametry
        self.monitor_interval = self.get_parameter('monitor_interval').get_parameter_value().double_value
        self.temp_warning = self.get_parameter('temperature_warning_threshold').get_parameter_value().double_value
        self.cpu_warning = self.get_parameter('cpu_warning_threshold').get_parameter_value().double_value
        self.memory_warning = self.get_parameter('memory_warning_threshold').get_parameter_value().double_value
        self.disk_warning = self.get_parameter('disk_warning_threshold').get_parameter_value().double_value
        
        # Publisher health
        self.health_pub = self.create_publisher(String, '/mss/node_health/system_monitor', 10)
        
        # Timer monitoringu
        self.monitor_timer = self.create_timer(self.monitor_interval, self.monitor_system)
        
        # === NOWY TIMER: Health reporting co 5 sekund ===
        self.health_timer = self.create_timer(5.0, self.publish_health)
        
        self.get_logger().info(f"System Monitor RPi uruchomiony. Interwał: {self.monitor_interval}s")
    
    def get_temperature(self):
        """Pobiera temperaturę CPU RPi."""
        try:
            # Różne sposoby na RPi
            temp_paths = [
                '/sys/class/thermal/thermal_zone0/temp',
                '/sys/devices/virtual/thermal/thermal_zone0/temp'
            ]
            
            for path in temp_paths:
                if os.path.exists(path):
                    with open(path, 'r') as f:
                        temp_millicelsius = int(f.read().strip())
                        return temp_millicelsius / 1000.0  # Konwersja na °C
            
            # Fallback: vcgencmd
            result = subprocess.run(['vcgencmd', 'measure_temp'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                temp_str = result.stdout.strip()
                temp_celsius = float(temp_str.replace('temp=', '').replace("'C", ''))
                return temp_celsius
                
        except Exception as e:
            self.get_logger().warn(f"Błąd odczytu temperatury: {e}")
        
        return None
    
    def get_gpio_status(self):
        """Sprawdza status GPIO."""
        try:
            # Sprawdź czy GPIO jest dostępne
            if os.path.exists('/sys/class/gpio'):
                return "OK"
            else:
                return "N/A"
        except Exception as e:
            return f"ERROR: {e}"
    
    def get_network_status(self):
        """Sprawdza status sieci."""
        try:
            # Sprawdź interfejsy sieciowe
            interfaces = psutil.net_if_addrs()
            active_interfaces = []
            
            for interface, addrs in interfaces.items():
                if interface != 'lo':  # Pomiń loopback
                    for addr in addrs:
                        if addr.family == 2:  # IPv4
                            active_interfaces.append(interface)
                            break
            
            if active_interfaces:
                return f"OK: {', '.join(active_interfaces)}"
            else:
                return "WARNING: Brak aktywnych interfejsów"
                
        except Exception as e:
            return f"ERROR: {e}"
    
    def get_usb_serial_status(self):
        """Sprawdza status USB i portów szeregowych."""
        try:
            # Sprawdź porty USB
            usb_devices = []
            if os.path.exists('/proc/bus/usb/devices'):
                with open('/proc/bus/usb/devices', 'r') as f:
                    content = f.read()
                    usb_devices = [line for line in content.split('\n') if 'T:' in line]
            
            # Sprawdź porty szeregowe
            serial_ports = []
            if os.path.exists('/dev'):
                serial_ports = [f for f in os.listdir('/dev') if f.startswith('ttyUSB') or f.startswith('ttyACM')]
            
            status = {
                'usb_devices': len(usb_devices),
                'serial_ports': serial_ports
            }
            
            return status
            
        except Exception as e:
            return f"ERROR: {e}"
    
    def monitor_system(self):
        """Główna funkcja monitoringu systemu."""
        try:
            # Zbierz metryki
            cpu_percent = psutil.cpu_percent(interval=0.1)
            memory = psutil.virtual_memory()
            disk = psutil.disk_usage('/')
            temperature = self.get_temperature()
            gpio_status = self.get_gpio_status()
            network_status = self.get_network_status()
            usb_serial_status = self.get_usb_serial_status()
            
            # Sprawdź ostrzeżenia
            warnings = []
            errors = []
            
            if temperature is not None:
                if temperature > self.temp_warning:
                    warnings.append(f"Temperatura wysoka: {temperature:.1f}°C")
                elif temperature > 60:
                    warnings.append(f"Temperatura podwyższona: {temperature:.1f}°C")
            
            if cpu_percent > self.cpu_warning:
                warnings.append(f"CPU wysokie: {cpu_percent:.1f}%")
            
            if memory.percent > self.memory_warning:
                warnings.append(f"RAM wysokie: {memory.percent:.1f}%")
            
            if disk.percent > self.disk_warning:
                warnings.append(f"Dysk pełny: {disk.percent:.1f}%")
            
            # Stwórz health report
            health_data = {
                'status': 'running',
                'timestamp': time.time(),
                'errors': errors,
                'warnings': warnings,
                'metrics': {
                    'cpu_usage_percent': cpu_percent,
                    'memory_usage_percent': memory.percent,
                    'memory_available_mb': memory.available / (1024*1024),
                    'disk_usage_percent': disk.percent,
                    'disk_free_gb': disk.free / (1024**3),
                    'temperature_celsius': temperature,
                    'gpio_status': gpio_status,
                    'network_status': network_status,
                    'usb_serial_status': usb_serial_status,
                    'uptime_seconds': time.time() - psutil.boot_time()
                }
            }
            
            # Opublikuj health
            health_msg = String()
            health_msg.data = json.dumps(health_data)
            self.health_pub.publish(health_msg)
            
        except Exception as e:
            self.get_logger().error(f"Błąd podczas monitoringu systemu: {e}")

    def publish_health(self):
        """Publikuje status zdrowia węzła system monitor."""
        try:
            # Sprawdź status timerów
            monitor_timer_status = "OK" if hasattr(self, 'monitor_timer') else "ERROR"
            health_timer_status = "OK" if hasattr(self, 'health_timer') else "ERROR"
            
            # Sprawdź status publisher'a
            health_pub_status = "OK" if hasattr(self, 'health_pub') else "ERROR"
            
            # Zbierz dane o błędach i ostrzeżeniach
            errors = []
            warnings = []
            
            if monitor_timer_status == "ERROR":
                errors.append("Timer monitoringu nieaktywny")
            if health_timer_status == "ERROR":
                errors.append("Timer health nieaktywny")
            if health_pub_status == "ERROR":
                errors.append("Publisher health nieaktywny")
            
            # Przygotuj dane health
            health_data = {
                'status': 'running' if not errors else 'error',
                'timestamp': time.time(),
                'monitor_timer_status': monitor_timer_status,
                'health_timer_status': health_timer_status,
                'health_pub_status': health_pub_status,
                'monitor_interval': self.monitor_interval,
                'errors': errors,
                'warnings': warnings,
                'cpu_usage': psutil.cpu_percent(),
                'memory_usage': psutil.virtual_memory().percent
            }
            
            # Opublikuj health status
            health_msg = String()
            health_msg.data = json.dumps(health_data)
            self.health_pub.publish(health_msg)
            
            # === DEBUG: Logowanie publikacji ===
            self.get_logger().info(f"📤 HEALTH PUBLISHED: {health_data['status']}")
            
        except Exception as e:
            self.get_logger().error(f"Błąd podczas publikowania health status: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
