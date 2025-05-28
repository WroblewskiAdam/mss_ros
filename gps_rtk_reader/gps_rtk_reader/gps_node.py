#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import socket
import base64
import serial
import threading
import time
from datetime import datetime, timezone # Do obsługi czasu

# Zaimportuj swoją customową wiadomość
# ZASTĄP 'twoj_pakiet_msgs.msg' i 'GpsRtk' właściwymi nazwami
from gps_rtk_msgs.msg import GpsRtk # Przykład: from my_interfaces.msg import GpsData


class GpsRtkNode(Node):
    def __init__(self):
        super().__init__('gps_rtk_node')

        # Deklaracja parametrów (z wartościami domyślnymi z Twojego skryptu)
        self.declare_parameter('serial_port', '/dev/ttyUSB0') # Zmień na COM4 jeśli Windows, ale w ROS lepiej ścieżki Linuxowe
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('ntrip_ip', 'system.asgeupos.pl')
        self.declare_parameter('ntrip_port', 8080)
        self.declare_parameter('ntrip_mountpoint', 'RTN4G_VRS_RTCM32') 
        self.declare_parameter('ntrip_user', 'pwmgr/adamwrb')
        self.declare_parameter('ntrip_password', 'Globus7142001')
        self.declare_parameter('gngga_ntrip_interval', 10.0) # sekundy
        self.declare_parameter('publish_frequency', 20.0) # Hz, jak często próbować publikować

        # Pobranie parametrów
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.ntrip_ip = self.get_parameter('ntrip_ip').get_parameter_value().string_value
        self.ntrip_port = self.get_parameter('ntrip_port').get_parameter_value().integer_value
        self.ntrip_mountpoint = self.get_parameter('ntrip_mountpoint').get_parameter_value().string_value
        self.ntrip_user = self.get_parameter('ntrip_user').get_parameter_value().string_value
        self.ntrip_password = self.get_parameter('ntrip_password').get_parameter_value().string_value
        self.gngga_ntrip_interval = self.get_parameter('gngga_ntrip_interval').get_parameter_value().double_value
        self.publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value

        # QoS dla danych sensorycznych
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, # Dla sensorów często BEST_EFFORT jest OK
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1 # Chcemy najnowsze dane
        )
        self.publisher_ = self.create_publisher(GpsRtk, 'gps_rtk_data', qos_profile)

        # Zmienne do przechowywania ostatnich danych GPS
        self.latest_gps_data = {
            'gps_utc_time_str': None, # hhmmss.ss
            'rtk_status': 0,        # Domyślnie brak fixa
            'latitude_deg': 0.0,
            'longitude_deg': 0.0,
            'altitude_m': 0.0,
            'speed_mps': 0.0,
            'heading_deg': 0.0,
            'timestamp_gngga': 0.0, # Czas ostatniej aktualizacji GNGGA
            'timestamp_gnvtg': 0.0  # Czas ostatniej aktualizacji GNVTG
        }
        self.data_lock = threading.Lock() # Do ochrony self.latest_gps_data

        # Zmienne do obsługi NTRIP i portu szeregowego
        self.rtk_serial = None
        self.ntrip_socket = None
        self.gngga_for_ntrip_initial = None
        self.last_gngga_to_ntrip_str = None
        self.last_gngga_send_time = 0.0
        self.stop_threads_event = threading.Event()

        # Inicjalizacja połączeń i wątków
        if self._initialize_serial() and self._initialize_ntrip():
            self.thread_rtk_reader = threading.Thread(target=self._rtk_reader_thread_func, daemon=True)
            self.thread_ntrip_reader = threading.Thread(target=self._ntrip_reader_thread_func, daemon=True)
            self.thread_rtk_reader.start()
            self.thread_ntrip_reader.start()
            self.get_logger().info("Wątki RTK i NTRIP uruchomione.")
        else:
            self.get_logger().error("Inicjalizacja nie powiodła się. Node nie będzie działać poprawnie.")
            # Można by tu rzucić wyjątek lub zamknąć node

        # Timer do publikowania danych
        self.publish_timer = self.create_timer(1.0 / self.publish_frequency, self._publish_gps_data_callback)
        self.get_logger().info(f"GPS RTK Node uruchomiony. Publikowanie na '/gps_rtk_data' z częstotliwością {self.publish_frequency} Hz.")

    def _initialize_serial(self):
        self.get_logger().info(f"Otwieranie portu szeregowego {self.serial_port} z prędkością {self.baud_rate}...")
        try:
            self.rtk_serial = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1) # Krótki timeout dla readline
            self.get_logger().info(f"Port szeregowy {self.serial_port} otwarty.")

            self.get_logger().info("Oczekiwanie na pierwszą poprawną wiadomość GNGGA z modułu RTK...")
            start_time = time.time()
            initial_gngga_found = False
            while time.time() - start_time < 30: # Czekaj max 30 sekund
                if self.stop_threads_event.is_set(): return False
                line_bytes = self.rtk_serial.readline()
                if line_bytes:
                    str_nmea = line_bytes.decode("ascii", errors='ignore').strip()
                    if str_nmea.startswith("$GNGGA"):
                        parts = str_nmea.split(',')
                        if len(parts) > 6 and len(parts[6]) and parts[6] != '0':
                            self.gngga_for_ntrip_initial = str_nmea + "\r\n"
                            self.last_gngga_to_ntrip_str = self.gngga_for_ntrip_initial
                            self.get_logger().info(f"Znaleziono początkowe GNGGA: {str_nmea}")
                            initial_gngga_found = True
                            return True
            if not initial_gngga_found:
                self.get_logger().error("Nie udało się uzyskać początkowej wiadomości GNGGA w ciągu 30s.")
                return False
        except serial.SerialException as e:
            self.get_logger().error(f"Błąd otwarcia lub konfiguracji portu szeregowego {self.serial_port}: {e}")
            return False
        return True

    def _initialize_ntrip(self):
        if not self.gngga_for_ntrip_initial:
            self.get_logger().error("Brak początkowej wiadomości GNGGA do inicjalizacji NTRIP.")
            return False

        self.ntrip_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ntrip_socket.settimeout(15.0) # Timeout dla operacji na gnieździe
        try:
            self.get_logger().info(f"Łączenie z serwerem NTRIP {self.ntrip_ip}:{self.ntrip_port}...")
            self.ntrip_socket.connect((self.ntrip_ip, self.ntrip_port))
            self.get_logger().info("Połączono z serwerem NTRIP.")

            user_pwd_encoded = base64.b64encode(bytes(self.ntrip_user + ':' + self.ntrip_password, 'utf-8')).decode("utf-8")
            http_request = (
                f"GET /{self.ntrip_mountpoint} HTTP/1.1\r\n"
                f"User-Agent: NTRIP ROS2Client/1.0\r\n"
                f"Authorization: Basic {user_pwd_encoded}\r\n"
                f"Connection: close\r\n\r\n"
            )
            self.get_logger().info("Wysyłanie żądania autoryzacji NTRIP...")
            self.ntrip_socket.sendall(http_request.encode())
            response = self.ntrip_socket.recv(2048) # Zwiększony bufor
            response_str = response.decode('ascii', errors='ignore')
            self.get_logger().info(f"Odpowiedź serwera NTRIP: {response_str.strip()}")

            if "ICY 200 OK" not in response_str:
                self.get_logger().error(f"Błąd autoryzacji NTRIP lub serwer nie jest gotowy. Odpowiedź: {response_str.strip()}")
                self.ntrip_socket.close()
                return False
            
            self.get_logger().info("Autoryzacja NTRIP pomyślna.")
            self.get_logger().info(f"Wysyłanie początkowej wiadomości GNGGA do NTRIP: {self.gngga_for_ntrip_initial.strip()}")
            self.ntrip_socket.sendall(self.gngga_for_ntrip_initial.encode())
            self.last_gngga_send_time = time.time()
            return True

        except socket.error as e:
            self.get_logger().error(f"Błąd połączenia z serwerem NTRIP: {e}")
            if self.ntrip_socket: self.ntrip_socket.close()
            return False
        except Exception as e:
            self.get_logger().error(f"Inny błąd podczas konfiguracji NTRIP: {e}")
            if self.ntrip_socket: self.ntrip_socket.close()
            return False

    def _rtk_reader_thread_func(self):
        self.get_logger().info("Wątek czytnika RTK uruchomiony.")
        while not self.stop_threads_event.is_set() and rclpy.ok():
            try:
                if not self.rtk_serial or not self.rtk_serial.is_open:
                    self.get_logger().warn("Port szeregowy nie jest otwarty. Próba ponownego otwarcia za 5s.")
                    time.sleep(5)
                    if not self._initialize_serial(): continue # Spróbuj ponownie otworzyć
                
                line_bytes = self.rtk_serial.readline()
                if line_bytes:
                    decoded_line = line_bytes.decode('ascii', errors='ignore').strip()
                    if not decoded_line: continue

                    current_host_time = self.get_clock().now().nanoseconds / 1e9 # Czas ROS
                    parts = decoded_line.split(',')
                    msg_type = parts[0]

                    with self.data_lock:
                        if msg_type == "$GNGGA" and len(parts) > 9:
                            try:
                                self.latest_gps_data['gps_utc_time_str'] = parts[1]
                                self.latest_gps_data['latitude_deg'] = self._nmea_to_decimal_degrees(parts[2], parts[3])
                                self.latest_gps_data['longitude_deg'] = self._nmea_to_decimal_degrees(parts[4], parts[5])
                                self.latest_gps_data['rtk_status'] = int(parts[6]) if parts[6] else 0
                                self.latest_gps_data['altitude_m'] = float(parts[9]) if parts[9] else 0.0
                                self.latest_gps_data['timestamp_gngga'] = current_host_time
                                self.last_gngga_to_ntrip_str = decoded_line + "\r\n" # Aktualizuj GNGGA do wysyłania
                            except (ValueError, IndexError) as e:
                                self.get_logger().warn(f"Błąd parsowania GNGGA '{decoded_line}': {e}")

                        elif msg_type == "$GNVTG" and len(parts) > 7:
                            try:
                                # Prędkość w km/h (parts[7]), chcemy m/s
                                speed_kph = float(parts[7]) if parts[7] and parts[8].upper() == 'K' else 0.0
                                self.latest_gps_data['speed_mps'] = speed_kph * (1000.0 / 3600.0)
                                # Kurs (heading)
                                self.latest_gps_data['heading_deg'] = float(parts[1]) if parts[1] and parts[2].upper() == 'T' else 0.0
                                self.latest_gps_data['timestamp_gnvtg'] = current_host_time
                            except (ValueError, IndexError) as e:
                                self.get_logger().warn(f"Błąd parsowania GNVTG '{decoded_line}': {e}")
                        # Możesz tu dodać logowanie innych wiadomości NMEA jeśli chcesz
                        # self.get_logger().debug(f"NMEA: {decoded_line}")


                    # Okresowe wysyłanie GNGGA do NTRIP
                    if self.last_gngga_to_ntrip_str and (time.time() - self.last_gngga_send_time > self.gngga_ntrip_interval):
                        if self.ntrip_socket:
                            try:
                                self.get_logger().info(f"NTRIP_UPDATE: Wysyłanie GNGGA: {self.last_gngga_to_ntrip_str.strip()}")
                                self.ntrip_socket.sendall(self.last_gngga_to_ntrip_str.encode('ascii'))
                                self.last_gngga_send_time = time.time()
                            except socket.error as se_ntrip:
                                self.get_logger().error(f"Błąd gniazda podczas wysyłania GNGGA do NTRIP: {se_ntrip}")
                                # Można rozważyć próbę ponownego połączenia z NTRIP
                                self.ntrip_socket.close() # Zamknij gniazdo, aby wymusić próbę re-inicjalizacji
                                self.ntrip_socket = None 
                                if not self._initialize_ntrip():
                                     self.get_logger().warn("Ponowna inicjalizacja NTRIP nie powiodła się.")
                            except Exception as e:
                                self.get_logger().error(f"Inny błąd wysyłania GNGGA: {e}")
                        else:
                            self.get_logger().warn("Próba wysłania GNGGA, ale gniazdo NTRIP nie jest aktywne. Próba re-inicjalizacji.")
                            if not self._initialize_ntrip(): # Jeśli gniazdo zostało zamknięte
                                self.get_logger().warn("Ponowna inicjalizacja NTRIP nie powiodła się.")


            except serial.SerialException as e:
                self.get_logger().error(f"Błąd portu szeregowego w wątku RTK: {e}")
                if self.rtk_serial and self.rtk_serial.is_open: self.rtk_serial.close()
                self.rtk_serial = None # Wymuś re-inicjalizację
                time.sleep(5) # Odczekaj przed próbą ponownego połączenia
            except Exception as e:
                self.get_logger().error(f"Nieoczekiwany błąd w wątku RTK: {e}", exc_info=True)
                time.sleep(1) # Krótka pauza
        self.get_logger().info("Wątek czytnika RTK zakończony.")


    def _ntrip_reader_thread_func(self):
        self.get_logger().info("Wątek czytnika NTRIP uruchomiony.")
        while not self.stop_threads_event.is_set() and rclpy.ok():
            try:
                if not self.ntrip_socket:
                    self.get_logger().warn("Gniazdo NTRIP nie jest aktywne. Oczekiwanie na re-inicjalizację.")
                    time.sleep(5) # Poczekaj na główny wątek lub wątek RTK, aby spróbować ponownie połączyć
                    if not self.ntrip_socket: # Jeśli nadal nie ma gniazda po przerwie
                         if not self._initialize_ntrip(): # Spróbuj zainicjować ręcznie
                              self.get_logger().warn("Ponowna inicjalizacja NTRIP w wątku NTRIP nie powiodła się.")
                              time.sleep(5) # Dłuższa przerwa
                              continue
                
                data = self.ntrip_socket.recv(4096)
                if data:
                    if self.rtk_serial and self.rtk_serial.is_open:
                        self.rtk_serial.write(data)
                        # self.get_logger().debug(f"NTRIP_RECV: Otrzymano {len(data)} bajtów korekty.")
                    else:
                        self.get_logger().warn("Otrzymano dane NTRIP, ale port szeregowy RTK nie jest otwarty.")
                else:
                    self.get_logger().warn("Połączenie NTRIP zamknięte przez serwer (recv zwrócił 0). Próba ponownego połączenia.")
                    self.ntrip_socket.close()
                    self.ntrip_socket = None
                    # Próba ponownego połączenia zostanie podjęta przez logikę w tym wątku lub _rtk_reader_thread_func
                    time.sleep(5) # Czekaj przed próbą
            except socket.timeout:
                self.get_logger().debug("Timeout podczas odbierania danych NTRIP. Kontynuuję...")
                continue
            except socket.error as e:
                self.get_logger().error(f"Błąd gniazda w wątku NTRIP: {e}")
                if self.ntrip_socket: self.ntrip_socket.close()
                self.ntrip_socket = None # Wymuś re-inicjalizację
                time.sleep(5)
            except serial.SerialException as e:
                 self.get_logger().error(f"Błąd portu szeregowego przy zapisie danych NTRIP: {e}")
                 if self.rtk_serial and self.rtk_serial.is_open: self.rtk_serial.close()
                 self.rtk_serial = None
                 time.sleep(5)
            except Exception as e:
                self.get_logger().error(f"Nieoczekiwany błąd w wątku NTRIP: {e}", exc_info=True)
                time.sleep(1)
        self.get_logger().info("Wątek czytnika NTRIP zakończony.")

    def _nmea_to_decimal_degrees(self, nmea_coord, hemisphere):
        if not nmea_coord:
            return 0.0
        try:
            coord = float(nmea_coord)
            degrees = int(coord / 100)
            minutes = coord - (degrees * 100)
            decimal_degrees = degrees + (minutes / 60.0)
            if hemisphere.upper() in ['S', 'W']:
                decimal_degrees *= -1.0
            return decimal_degrees
        except ValueError:
            self.get_logger().warn(f"Nie można przekonwertować współrzędnej NMEA: {nmea_coord}")
            return 0.0

    def _convert_gps_utc_to_ros_time(self, utc_time_str):
        """Konwertuje czas UTC z GNGGA (hhmmss.ss) na ROS Time (builtin_interfaces.msg.Time)"""
        if not utc_time_str or '.' not in utc_time_str:
            return self.get_clock().now().to_msg() # Zwróć aktualny czas ROS jako fallback

        try:
            # Użyj dzisiejszej daty systemowej UTC, połącz z czasem GPS
            now_utc = datetime.now(timezone.utc)
            
            hour = int(utc_time_str[0:2])
            minute = int(utc_time_str[2:4])
            second = int(utc_time_str[4:6])
            subsecond_str = utc_time_str.split('.')[1]
            nanosecond = int(subsecond_str.ljust(9, '0')[:9]) # Uzupełnij do 9 cyfr dla nanosekund

            # Stwórz obiekt datetime z dzisiejszą datą i czasem GPS
            gps_datetime = now_utc.replace(hour=hour, minute=minute, second=second, microsecond=nanosecond // 1000, tzinfo=timezone.utc)
            
            ros_time = GpsRtk().header.stamp # Uzyskaj pusty obiekt Time
            ros_time.sec = int(gps_datetime.timestamp())
            ros_time.nanosec = gps_datetime.microsecond * 1000 # nanosecond od początku sekundy
            return ros_time
        except ValueError as e:
            self.get_logger().warn(f"Błąd konwersji czasu GPS UTC '{utc_time_str}': {e}")
            return self.get_clock().now().to_msg()


    def _publish_gps_data_callback(self):
        msg = GpsRtk()
        current_ros_time = self.get_clock().now()

        with self.data_lock:
            # Sprawdź świeżość danych (np. nie starsze niż 0.5 sekundy)
            # To ważne, bo GNGGA i GNVTG mogą przychodzić z lekkim opóźnieniem względem siebie.
            # Tutaj uproszczenie - publikujemy jeśli GNGGA jest dostępne.
            # W bardziej zaawansowanym podejściu można czekać na oba "świeże" komunikaty.
            if self.latest_gps_data['timestamp_gngga'] > 0: # Jeśli GNGGA było kiedykolwiek odebrane
                msg.header.stamp = current_ros_time.to_msg()
                msg.header.frame_id = "gps_link" # lub inna odpowiednia ramka odniesienia

                msg.gps_time = self._convert_gps_utc_to_ros_time(self.latest_gps_data['gps_utc_time_str'])
                msg.rtk_status = self.latest_gps_data['rtk_status']
                msg.latitude_deg = self.latest_gps_data['latitude_deg']
                msg.longitude_deg = self.latest_gps_data['longitude_deg']
                msg.altitude_m = self.latest_gps_data['altitude_m']
                
                # Użyj danych z GNVTG jeśli są świeże, w przeciwnym razie mogą być zerowe lub stare
                # Dla uproszczenia zakładamy, że jeśli GNGGA jest, to GNVTG też wkrótce będzie
                # i używamy ostatniej znanej wartości.
                msg.speed_mps = self.latest_gps_data['speed_mps']
                msg.heading_deg = self.latest_gps_data['heading_deg']

                self.publisher_.publish(msg)
                # self.get_logger().debug(f"Opublikowano dane GPS: Lat {msg.latitude_deg}, Lon {msg.longitude_deg}")
            # else:
                # self.get_logger().debug("Brak danych GNGGA do opublikowania.")


    def destroy_node(self):
        self.get_logger().info("Zamykanie node'a GPS RTK...")
        self.stop_threads_event.set() # Sygnalizuj wątkom, aby się zakończyły

        if hasattr(self, 'thread_rtk_reader') and self.thread_rtk_reader.is_alive():
            self.thread_rtk_reader.join(timeout=2.0)
        if hasattr(self, 'thread_ntrip_reader') and self.thread_ntrip_reader.is_alive():
            self.thread_ntrip_reader.join(timeout=2.0)
        
        if self.rtk_serial and self.rtk_serial.is_open:
            self.rtk_serial.close()
            self.get_logger().info("Port szeregowy RTK zamknięty.")
        if self.ntrip_socket:
            try:
                self.ntrip_socket.shutdown(socket.SHUT_RDWR)
            except socket.error: pass # Ignoruj, jeśli już zamknięte
            self.ntrip_socket.close()
            self.get_logger().info("Połączenie NTRIP zamknięte.")
        
        super().destroy_node()
        self.get_logger().info("Node GPS RTK zamknięty.")


def main(args=None):
    rclpy.init(args=args)
    gps_rtk_node = None
    try:
        gps_rtk_node = GpsRtkNode()
        rclpy.spin(gps_rtk_node)
    except KeyboardInterrupt:
        print("Przerwano przez użytkownika (Ctrl+C)")
    except Exception as e:
        if gps_rtk_node:
            gps_rtk_node.get_logger().fatal(f"Krytyczny błąd w main: {e}", exc_info=True)
        else:
            print(f"Krytyczny błąd przed inicjalizacją node'a: {e}")
    finally:
        if gps_rtk_node:
            gps_rtk_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()