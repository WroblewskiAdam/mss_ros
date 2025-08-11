# Wiadomość agregująca kluczowe dane diagnostyczne

std_msgs/Header header

# 1. Dane z gps_rtk_data_filtered (ciągnik)
my_robot_interfaces/GpsRtk tractor_gps_filtered

# 2. Dane z choppera
my_robot_interfaces/GpsRtk chopper_gps

# 3. Dane od serwa (aktualna pozycja)
my_robot_interfaces/StampedInt32 servo_position

# 4. Status komunikacji BT (true - OK, false - Błąd/Brak połączenia)
bool bt_status

# 5. Dane z topicku /gears
my_robot_interfaces/Gear tractor_gear

# 6. Dane z systemu sterowania
std_msgs/Float64 target_speed
my_robot_interfaces/DistanceMetrics relative_position