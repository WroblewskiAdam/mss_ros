from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    """Uruchamia wszystkie niezbędne węzły dla systemu MSS."""
    return LaunchDescription([
        
        # === Węzły sensorów i komunikacji ===
        Node(
            package='gps_rtk_reader',
            executable='gps_rtk_node',
            name='gps_rtk_node',
            output='screen',
            emulate_tty=True
        ),
        # Node(
        #     package='bt_comm',
        #     executable='bt_receiver_node',
        #     name='bt_receiver_node',
        #     output='screen',
        #     emulate_tty=True
        # ),
        Node(
            package='mqtt_comm',
            executable='mqtt_node',
            name='mqtt_node',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='gear_reader',
            executable='gear_reader_node',
            name='gear_reader_node',
            output='screen',
            emulate_tty=True,
        ),
        # === Węzły wykonawcze (aktuatory) ===
        Node(
            package='servo_controller',
            executable='servo_node',
            name='servo_controller',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='gear_controller',
            executable='gear_shifter',
            name='gear_shifter',
            output='screen',
            emulate_tty=True
        ),

        # === Węzły logiki i sterowania ===
        Node(
            package='mss_filters',
            executable='tractor_filter_node',
            name='tractor_filter_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='mss_filters',
            executable='chopper_filter_node',
            name='chopper_filter_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='speed_controller',
            executable='speed_controller_node',
            name='speed_controller_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='relative_position_computer',
            executable='relative_computer_node',
            name='relative_computer_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='gear_manager',
            executable='gear_manager_node',
            name='gear_manager_node',
            output='screen',
            emulate_tty=True,
        ),
        
        # === Regulator Pozycji (Autopilot) ===
        Node(
            package='position_controller',
            executable='position_controller_node',
            name='position_controller_node',
            output='screen',
            emulate_tty=True,
        ),
        
        # === Węzły pomocnicze (kluczowe dla UI) ===
        Node(
            package='mss_diagnostics',
            executable='diagnostics_node',
            name='diagnostics_node',
            output='screen',
            emulate_tty=True,
        ),
        
        # === Data Logger ===
        Node(
            package='data_logger',
            executable='mss_data_logger_node',
            name='mss_data_logger_node',
            output='screen',
            emulate_tty=True,
        ),
                    
                                # === System Monitor RPi ===
            TimerAction(
                period=5.0,  # 3 sekundy delay
                actions=[
                    Node(
                        package='mss_system_monitor',
                        executable='system_monitor_node',
                        name='system_monitor_node',
                        output='screen',
                        emulate_tty=True,
                    )
                ]
            ),
            
            # === NOWY SYSTEM: Health Monitor (zamiast watchdog'a) ===
            TimerAction(
                period=10.0,  # 5 sekund delay
                actions=[
                    Node(
                        package='mss_health_monitor',
                        executable='health_monitor_node',
                        name='mss_health_monitor_node',
                        output='screen',
                        emulate_tty=True,
                    )
                ]
            ),
                ])
