#!/usr/bin/env python3

"""
Skrypt testowy dla MQTT Chopper Receiver
Sprawdza połączenie z brokerem i publikuje testowe dane
"""

import rclpy
from rclpy.node import Node
import json
import time
import paho.mqtt.client as mqtt

class MQTTTester(Node):
    def __init__(self):
        super().__init__('mqtt_tester')
        
        # Parametry MQTT
        self.mqtt_host = 'mss-mqtt.ddns.net'
        self.mqtt_port = 1883
        self.mqtt_topic = 'chopper/gps'
        
        # Klient MQTT
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_disconnect = self.on_disconnect
        self.mqtt_client.on_publish = self.on_publish
        
        # Timer do publikacji testowych danych
        self.publish_timer = self.create_timer(2.0, self.publish_test_data)
        
        self.get_logger().info("MQTT Tester uruchomiony")
        self.get_logger().info(f"Broker: {self.mqtt_host}:{self.mqtt_port}")
        self.get_logger().info(f"Topic: {self.mqtt_topic}")
        
        # Połącz z brokerem
        self.connect_to_broker()
    
    def connect_to_broker(self):
        """Połączenie z brokerem MQTT"""
        try:
            self.get_logger().info(f"Łączenie z brokerem: {self.mqtt_host}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"Błąd połączenia: {e}")
    
    def on_connect(self, client, userdata, flags, rc):
        """Callback połączenia MQTT"""
        if rc == 0:
            self.get_logger().info("✅ Połączono z brokerem MQTT")
        else:
            self.get_logger().error(f"❌ Błąd połączenia: {rc}")
    
    def on_disconnect(self, client, userdata, rc):
        """Callback rozłączenia MQTT"""
        self.get_logger().warn(f"⚠️  Rozłączono z brokerem: {rc}")
    
    def on_publish(self, client, userdata, mid):
        """Callback publikacji MQTT"""
        self.get_logger().info(f"📤 Opublikowano wiadomość: {mid}")
    
    def publish_test_data(self):
        """Publikacja testowych danych GPS"""
        if not self.mqtt_client.is_connected():
            self.get_logger().warn("Brak połączenia z brokerem")
            return
        
        # Generuj testowe dane GPS
        test_data = {
            "latitude": 52.123456 + (time.time() % 100) * 0.000001,
            "longitude": 21.123456 + (time.time() % 100) * 0.000001,
            "altitude": 150.5,
            "speed": 2.5 + (time.time() % 10) * 0.1,
            "heading": (time.time() * 10) % 360,
            "rtk_status": 4
        }
        
        # Konwertuj na JSON
        json_data = json.dumps(test_data)
        
        # Opublikuj
        result = self.mqtt_client.publish(self.mqtt_topic, json_data)
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().info(f"📡 Wysłano testowe dane: {test_data}")
        else:
            self.get_logger().error(f"❌ Błąd publikacji: {result.rc}")
    
    def destroy_node(self):
        """Zamknij połączenie MQTT"""
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    mqtt_tester = MQTTTester()
    
    try:
        rclpy.spin(mqtt_tester)
    except KeyboardInterrupt:
        pass
    finally:
        mqtt_tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
