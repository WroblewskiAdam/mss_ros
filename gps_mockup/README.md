# GPS Mockup Package

Pakiet symulujący dane GPS dla systemu MSS (Maszyna Sieczkarnia System).

## Opis

Ten pakiet zawiera węzeł `gps_mockup_node`, który symuluje realistyczne dane GPS RTK dla:
- **Ciągnika** - publikuje na topiku `/gps_rtk_data`
- **Sieczkarni** - publikuje na topiku `/gps_rtk_data/chopper`

## Funkcjonalności

### Symulacja Ruchu
- Ciągnik porusza się z prędkością 2 m/s (~7.2 km/h)
- Sieczkarnia jedzie 5m za ciągnikiem
- Realistyczne zakręty i odchylenia (symulacja nierówności terenu)
- Ruch głównie na północ z sinusoidalnymi odchyleniami

### Dane GPS
- Współrzędne geograficzne (Warszawa jako punkt bazowy)
- Kurs (heading) w stopniach
- Prędkość w m/s
- Wysokość (120m ±2m)
- Status RTK (SPS, DGPS, FIX, FLOAT)
- Czas GPS

### Parametry Konfiguracyjne
- `publish_frequency_hz` - częstotliwość publikacji (domyślnie 10 Hz)
- `tractor_speed_mps` - prędkość ciągnika (domyślnie 2.0 m/s)
- `chopper_speed_mps` - prędkość sieczkarni (domyślnie 1.9 m/s)
- `chopper_offset_m` - odległość sieczkarni za ciągnikiem (domyślnie 5.0 m)
- `simulation_area_lat` - szerokość geograficzna punktu bazowego (domyślnie 52.2297)
- `simulation_area_lon` - długość geograficzna punktu bazowego (domyślnie 21.0122)

## Uruchamianie

### 1. Budowanie pakietu
```bash
cd /home/pi/mss_ros
colcon build --packages-select gps_mockup
source install/setup.bash
```

### 2. Uruchomienie samego węzła mockup
```bash
ros2 run gps_mockup mockup_node
```

### 3. Uruchomienie całego systemu z mockup danymi
```bash
cd /home/pi/mss_ros/src
./mss_startup_mockup.sh
```

### 4. Uruchomienie przez launch file
```bash
ros2 launch mss_bringup all_nodes_mockup.launch.py
```

## Testowanie

### Sprawdzenie topików
```bash
# Lista aktywnych topików
ros2 topic list

# Sprawdzenie danych ciągnika
ros2 topic echo /gps_rtk_data

# Sprawdzenie danych sieczkarni
ros2 topic echo /gps_rtk_data/chopper

# Sprawdzenie częstotliwości
ros2 topic hz /gps_rtk_data
```

### Sprawdzenie węzłów
```bash
# Lista aktywnych węzłów
ros2 node list

# Informacje o węźle mockup
ros2 node info /gps_mockup_node
```

## Integracja z Web Interface

Po uruchomieniu systemu z mockup danymi, web interface będzie wyświetlał:
- Symulowane pozycje ciągnika i sieczkarni
- Realistyczne dane GPS w czasie rzeczywistym
- Wizualizację względnej pozycji pojazdów
- Wszystkie funkcje sterowania i monitoringu

## Różnice względem prawdziwego systemu

### Wyłączone węzły w trybie mockup:
- `gps_rtk_node` - prawdziwy odczyt GPS RTK
- `bt_receiver_node` - komunikacja Bluetooth z sieczkarnią

### Aktywne węzły:
- `gps_mockup_node` - symulacja danych GPS
- `gear_reader_node` - odczyt biegów (może być symulowany)
- Wszystkie węzły sterowania i monitoringu

## Konfiguracja

Parametry można zmienić w launch file lub przez parametry ROS **w czasie rzeczywistym**:
```bash
# Zmiana prędkości ciągnika (natychmiastowa)
ros2 param set /gps_mockup_node tractor_speed_mps 3.0

# Zmiana prędkości sieczkarni (natychmiastowa)
ros2 param set /gps_mockup_node chopper_speed_mps 2.8

# Zmiana odległości sieczkarni (natychmiastowa)
ros2 param set /gps_mockup_node chopper_offset_m 7.0

# Zmiana częstotliwości publikacji (natychmiastowa)
ros2 param set /gps_mockup_node publish_frequency_hz 20.0
```

## Troubleshooting

### Problem: Węzeł się nie uruchamia
- Sprawdź czy pakiet jest zbudowany: `colcon build --packages-select gps_mockup`
- Sprawdź czy zależności są zainstalowane: `rosdep install --from-paths src --ignore-src -r -y`

### Problem: Brak danych na topikach
- Sprawdź czy węzeł jest aktywny: `ros2 node list`
- Sprawdź logi węzła: `ros2 node info /gps_mockup_node`

### Problem: Web interface nie pokazuje danych
- Sprawdź czy ROS Bridge jest uruchomiony
- Sprawdź czy węzeł `diagnostics_node` jest aktywny
- Sprawdź połączenie WebSocket w przeglądarce
