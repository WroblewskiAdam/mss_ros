import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from datetime import datetime

def generate_launch_description():
    # --- Konfiguracja ---

    # 1. Lista tematów do nagrania w pliku .bag
    topics_to_record = [
        '/gps_rtk_data',
        '/gps_rtk_data_filtered',
        '/imu/data_raw',
        '/imu/mag',
        '/servo/set_angle',
        '/servo/position',
        '/target_speed',
        # Warto dodać transformacje, są bardzo przydatne
        '/tf',
        '/tf_static'
    ]

    # 2. Ścieżka do zapisu plików .bag
    # Stworzenie unikalnej nazwy pliku .bag ze znacznikiem czasu
    bag_filename = 'mss_rosbag_' + datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_filepath = os.path.join(
        os.path.expanduser('~'),
        'ros2_bags', # Zapis w katalogu domowym/ros2_bags
        bag_filename
    )

    # --- Akcje do wykonania ---

    # Akcja 1: Uruchomienie Twojego węzła do zapisu w .csv
    csv_logger_node = Node(
        package='data_logger',
        executable='logger_node',
        name='data_logger_node_sync',
        output='screen',
        emulate_tty=True
    )

    # Akcja 2: Uruchomienie procesu nagrywania .bag
    record_bag_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_filepath] + topics_to_record,
        output='screen'
    )

    # Zwracamy opis całej logiki uruchomienia
    return LaunchDescription([
        csv_logger_node,
        record_bag_process
    ])