std_msgs/Header header       # Standardowy nagłówek ROS z czasem i frame_id
builtin_interfaces/Time gps_time # Czas UTC odczytany z GPS
uint8 rtk_status             # Status RTK (0: brak, 1: SPS, 2: DGPS, 4: RTK Fixed, 5: RTK Float etc.)
float64 latitude_deg         # Szerokość geograficzna w stopniach dziesiętnych
float64 longitude_deg        # Długość geograficzna w stopniach dziesiętnych
float64 altitude_m           # Wysokość n.p.m. w metrach
float64 speed_mps            # Prędkość w m/s
float64 heading_deg          # Kurs (orientacja) w stopniach (0-360)