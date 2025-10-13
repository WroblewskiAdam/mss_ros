import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from datetime import datetime

def generate_launch_description():
    # --- Konfiguracja ---

    # 1. Lista tematów do nagrania w pliku .bag
    topics_to_record = [
        '/gps_rtk_data/tractor',
        '/gps_rtk_data/tractor_filtered',
        '/gps_rtk_data/chopper',
        '/gps_rtk_data/chopper_filtered',
        '/servo/position',
        '/gears',
        '/distance_metrics',
        '/speed_controller/state',
        '/target_speed',
        '/target_position',
        '/autopilot/status',
        # Transformacje - bardzo przydatne
        '/tf',
        '/tf_static'
    ]

    # 2. Ścieżka do zapisu plików .bag
    # Stworzenie unikalnej nazwy pliku .bag ze znacznikiem czasu
    bag_filename = 'bag_system_log_' + datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_filepath = os.path.join(
        os.path.expanduser('~'),
        'mss_ros/src/logs_mss',  # Ten sam katalog co CSV
        bag_filename
    )

    # --- Akcja do wykonania ---

    # Uruchomienie procesu nagrywania .bag
    record_bag_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_filepath] + topics_to_record,
        output='screen'
    )

    # Zwracamy opis całej logiki uruchomienia
    return LaunchDescription([
        record_bag_process
    ])
