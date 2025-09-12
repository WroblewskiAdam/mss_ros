# Dokumentacja Systemu MSS (Maszyna Sieczkarnia System)

## Przegląd Systemu

System MSS to zaawansowany system synchronizacji prędkości i pozycji ciągnika rolniczego z sieczkarnią polową podczas zbiorów kukurydzy. System wykorzystuje technologię ROS2 (Robot Operating System 2) do komunikacji między komponentami i zapewnia precyzyjną kontrolę prędkości oraz pozycjonowanie względne pojazdów.

## Architektura Systemu

### Główne Komponenty

1. **System GPS RTK** - precyzyjne pozycjonowanie obu pojazdów
2. **Komunikacja Bluetooth** - łączność z sieczkarnią
3. **System Sterowania** - regulacja prędkości i pozycji
4. **Interfejs Operatora** - aplikacja webowa do monitorowania i kontroli
5. **System Monitoringu** - diagnostyka i health monitoring

## Struktura Pakietów ROS2

### 1. Pakiety Sensorów i Komunikacji

#### `gps_rtk_reader`
- **Węzeł**: `gps_rtk_node`
- **Funkcja**: Odczyt danych GPS RTK z ciągnika
- **Topiki publikowane**:
  - `/gps_rtk_data` - surowe dane GPS ciągnika
- **Wiadomości**: `my_robot_interfaces/GpsRtk`

#### `bt_comm`
- **Węzeł**: `bluetooth_receiver_node`
- **Funkcja**: Komunikacja Bluetooth z sieczkarnią
- **Topiki publikowane**:
  - `/gps_rtk_data/chopper` - dane GPS sieczkarni
- **Parametry**:
  - `bt_port`: port Bluetooth (domyślnie 1)
  - `connection_timeout`: timeout połączenia (0.5s)
  - `health_report_interval`: interwał raportów (5s)

#### `gps_mockup`
- **Węzeł**: `gps_mockup_node`
- **Funkcja**: Symulacja danych GPS dla testów
- **Topiki publikowane**:
  - `/gps_rtk_data` - symulowane dane ciągnika
  - `/gps_rtk_data/chopper` - symulowane dane sieczkarni
- **Parametry**:
  - `publish_frequency_hz`: częstotliwość publikacji (10 Hz)
  - `tractor_speed_mps`: prędkość ciągnika (2.0 m/s)
  - `chopper_speed_mps`: prędkość sieczkarni (1.9 m/s)
  - `chopper_offset_m`: odległość sieczkarni (5.0 m)

#### `gear_reader`
- **Węzeł**: `gear_reader_node`
- **Funkcja**: Odczyt stanu biegów i sprzęgła
- **Topiki publikowane**:
  - `/gears` - stan biegów i sprzęgła
- **GPIO**: Piny 5 (sprzęgło), 19,13,26,6 (biegi 1-4)

### 2. Pakiety Sterowania

#### `speed_controller`
- **Węzły**:
  - `speed_filter_node` - filtrowanie danych prędkości
  - `speed_controller_node` - regulator prędkości PID
- **Funkcje**:
  - Filtrowanie szumów GPS (filtr Butterworth)
  - Regulacja prędkości z algorytmem PID
  - Sterowanie serwem na podstawie błędu prędkości
- **Topiki**:
  - Subskrypcje: `/gps_rtk_data_filtered`, `/target_speed`, `/gears`
  - Publikacje: `/servo/set_angle`, `/speed_controller/state`
- **Parametry PID**:
  - `kp`: wzmocnienie proporcjonalne (10.0)
  - `ki`: wzmocnienie całkowe (20.0)
  - `kd`: wzmocnienie różniczkowe (0.0)

#### `servo_controller`
- **Węzeł**: `servo_controller`
- **Funkcja**: Sterowanie serwem silnika
- **Sprzęt**: PCA9685 (16-kanałowy sterownik PWM)
- **Topiki**:
  - Subskrypcje: `/servo/set_angle`
  - Publikacje: `/servo/position`
- **Usługi**:
  - `/servo/set_manual_mode` - przełączanie trybu ręcznego/automatycznego
- **Funkcje**:
  - Płynny ruch serwa (750°/s)
  - Watchdog timeout (0.2s)
  - Tryb ręczny i automatyczny

#### `gear_controller`
- **Węzeł**: `gear_shifter`
- **Funkcja**: Sterowanie zmianą biegów
- **GPIO**: Piny 25 (bieg w górę), 20 (bieg w dół)
- **Usługi**:
  - `/gear_shift_up` - zmiana biegu w górę
  - `/gear_shift_down` - zmiana biegu w dół

#### `gear_manager`
- **Węzeł**: `gear_manager_node`
- **Funkcja**: Automatyczne zarządzanie biegami
- **Logika**: Decyzje na podstawie prędkości zadanej
- **Parametry**:
  - `powershift_max_speeds`: maksymalne prędkości biegów [2.9, 3.6, 4.1, 5.6]
  - `upshift_threshold_percent`: próg zmiany w górę (0.95)
  - `downshift_threshold_percent`: próg zmiany w dół (0.85)

### 3. Pakiety Obliczeniowe

#### `relative_position_computer`
- **Węzeł**: `relative_computer_node`
- **Funkcja**: Obliczanie pozycji względnej pojazdów
- **Algorytmy**:
  - Konwersja współrzędnych geograficznych na ENU
  - Obliczanie odległości wzdłużnej i poprzecznej
  - Synchronizacja czasowa danych GPS
- **Topiki**:
  - Subskrypcje: `/gps_rtk_data`, `/gps_rtk_data/chopper`
  - Publikacje: `/distance_metrics`

### 4. Pakiety Monitoringu i Diagnostyki

#### `mss_diagnostics`
- **Węzeł**: `diagnostics_node`
- **Funkcja**: Agregacja danych diagnostycznych
- **Topiki publikowane**:
  - `/diagnostics` - skonsolidowane dane systemu
- **Dane agregowane**:
  - GPS ciągnika i sieczkarni
  - Pozycja serwa
  - Stan biegów i sprzęgła
  - Prędkość zadana
  - Pozycja względna

#### `mss_health_monitor`
- **Węzeł**: `mss_health_monitor_node`
- **Funkcja**: Monitoring zdrowia wszystkich węzłów
- **Topiki publikowane**:
  - `/mss/system_status` - ogólny status systemu
  - `/mss/node_status` - status poszczególnych węzłów
  - `/mss/health_alerts` - alerty o problemach

#### `mss_system_monitor`
- **Węzeł**: `system_monitor_node`
- **Funkcja**: Monitoring zasobów Raspberry Pi
- **Metryki**:
  - CPU, RAM, temperatura
  - Status GPIO, sieci, USB/Serial
  - Uptime systemu

### 5. Interfejs Operatora

#### `operator_interface`
- **Aplikacja webowa** z interfejsem użytkownika
- **Komunikacja**: ROS Bridge WebSocket (port 9090)
- **Serwer**: HTTP server (port 8080)
- **Funkcje**:
  - Wizualizacja pozycji pojazdów
  - Kontrola prędkości i parametrów PID
  - Monitoring systemu i health
  - Sterowanie serwem i biegami
  - Wykresy w czasie rzeczywistym

## Topiki ROS2

### Główne Topiki Danych

| Topik | Typ Wiadomości | Opis |
|-------|----------------|------|
| `/gps_rtk_data` | `GpsRtk` | Surowe dane GPS ciągnika |
| `/gps_rtk_data_filtered` | `GpsRtk` | Filtrowane dane GPS ciągnika |
| `/gps_rtk_data/chopper` | `GpsRtk` | Dane GPS sieczkarni |
| `/gears` | `Gear` | Stan biegów i sprzęgła |
| `/target_speed` | `Float64` | Prędkość zadana |
| `/servo/set_angle` | `StampedInt32` | Komenda kąta serwa |
| `/servo/position` | `StampedInt32` | Aktualna pozycja serwa |
| `/distance_metrics` | `DistanceMetrics` | Odległości względne |
| `/diagnostics` | `DiagnosticData` | Dane diagnostyczne |

### Topiki Sterowania

| Topik | Typ Wiadomości | Opis |
|-------|----------------|------|
| `/speed_controller/state` | `SpeedControllerState` | Stan regulatora PID |
| `/mss/system_status` | `String` | Status systemu |
| `/mss/node_status` | `String` | Status węzłów |
| `/mss/health_alerts` | `String` | Alerty zdrowia |

### Topiki Health Monitoring

| Topik | Opis |
|-------|------|
| `/mss/node_health/gps_rtk_node` | Health GPS RTK |
| `/mss/node_health/bt_receiver_node` | Health Bluetooth |
| `/mss/node_health/gear_reader_node` | Health odczytu biegów |
| `/mss/node_health/servo_controller` | Health serwa |
| `/mss/node_health/gear_shifter` | Health sterowania biegami |
| `/mss/node_health/speed_filter_node` | Health filtru prędkości |
| `/mss/node_health/speed_controller_node` | Health regulatora |
| `/mss/node_health/relative_computer_node` | Health obliczeń pozycji |
| `/mss/node_health/gear_manager_node` | Health menedżera biegów |
| `/mss/node_health/diagnostics_node` | Health diagnostyki |
| `/mss/node_health/system_monitor` | Health monitora systemu |
| `/mss/node_health/mss_health_monitor_node` | Health monitora zdrowia |

## Usługi ROS2

| Usługa | Typ | Opis |
|--------|-----|------|
| `/speed_controller/set_enabled` | `SetBool` | Włączanie/wyłączanie autopilota |
| `/gear_shift_up` | `SetBool` | Zmiana biegu w górę |
| `/gear_shift_down` | `SetBool` | Zmiana biegu w dół |
| `/servo/set_manual_mode` | `SetBool` | Tryb ręczny serwa |
| `/speed_controller_node/set_parameters` | `SetParameters` | Ustawianie parametrów PID |

## Wiadomości ROS2

### `GpsRtk.msg`
```
std_msgs/Header header
builtin_interfaces/Time gps_time
uint8 rtk_status
float64 latitude_deg
float64 longitude_deg
float64 altitude_m
float64 speed_mps
float64 heading_deg
```

### `DiagnosticData.msg`
```
std_msgs/Header header
GpsRtk tractor_gps_filtered
GpsRtk chopper_gps
StampedInt32 servo_position
bool bt_status
Gear tractor_gear
Float64 target_speed
DistanceMetrics relative_position
```

### `DistanceMetrics.msg`
```
std_msgs/Header header
float64 distance_straight
float64 distance_longitudinal
float64 distance_lateral
```

## Komunikacja Web Interface

### ROS Bridge
- **Protokół**: WebSocket
- **Port**: 9090
- **URL**: `ws://192.168.1.77:9090`
- **Biblioteka**: roslib.js

### Subskrypcje Web Interface

| Topik | Funkcja |
|-------|---------|
| `/diagnostics` | Dane diagnostyczne |
| `/speed_controller/state` | Wykres regulatora |
| `/mss/system_status` | Status systemu |
| `/mss/health_alerts` | Alerty |
| `/mss/node_health/*` | Health węzłów |

### Publikacje Web Interface

| Topik | Funkcja |
|-------|---------|
| `/target_speed` | Ustawianie prędkości |
| `/servo/set_angle` | Sterowanie serwem |

### Usługi Web Interface

| Usługa | Funkcja |
|--------|---------|
| `/speed_controller/set_enabled` | Autopilot |
| `/gear_shift_up/down` | Biegi |
| `/servo/set_manual_mode` | Tryb serwa |
| `/speed_controller_node/set_parameters` | Parametry PID |

## Struktura Plików

```
mss_ros/src/
├── bt_comm/                    # Komunikacja Bluetooth
├── data_logger/                # Logowanie danych
├── gear_controller/            # Sterowanie biegami
├── gear_manager/               # Automatyczne zarządzanie biegami
├── gear_reader/                # Odczyt biegów
├── gps_mockup/                 # Symulacja GPS
├── gps_rtk_msgs/               # Wiadomości GPS
├── gps_rtk_reader/             # Odczyt GPS RTK
├── imu_reader/                 # Odczyt IMU
├── mss_bringup/                # Launch files
├── mss_diagnostics/            # Diagnostyka
├── mss_health_monitor/         # Monitoring zdrowia
├── mss_system_monitor/         # Monitoring systemu
├── mss_visualization/          # Wizualizacja
├── my_robot_interfaces/        # Interfejsy wiadomości
├── operator_interface/         # Interfejs webowy
│   └── web/
│       ├── index.html          # Główna strona
│       ├── main.js             # Logika JavaScript
│       ├── style.css           # Style CSS
│       └── start_interface.sh  # Skrypt uruchamiający
├── relative_position_computer/ # Obliczenia pozycji
├── servo_controller/           # Sterowanie serwem
└── speed_controller/           # Regulacja prędkości
```

## Uruchamianie Systemu

### Tryb Mockup (Symulacja)
```bash
cd /home/pi/mss_ros/src
./mss_startup_mockup.sh
```

### Tryb Produkcyjny
```bash
cd /home/pi/mss_ros/src
./mss_startup.sh
```

### Web Interface
```bash
cd operator_interface/web
./start_interface.sh
```

## Konfiguracja

### Parametry Kluczowe

#### Regulator Prędkości
- `kp`: 10.0 (wzmocnienie proporcjonalne)
- `ki`: 20.0 (wzmocnienie całkowe)
- `kd`: 0.0 (wzmocnienie różniczkowe)
- `v_idle`: 1.3359 (prędkość jałowa)

#### Filtrowanie GPS
- `filter_cutoff_hz`: 0.8 (częstotliwość odcięcia)
- `filter_order`: 2 (rząd filtru)

#### Zarządzanie Biegami
- `powershift_max_speeds`: [2.9, 3.6, 4.1, 5.6]
- `upshift_threshold_percent`: 0.95
- `downshift_threshold_percent`: 0.85

## Monitoring i Diagnostyka

### Health Monitoring
- Każdy węzeł publikuje status zdrowia co 5 sekund
- Centralny monitor agreguje dane z wszystkich węzłów
- Alerty o problemach publikowane na `/mss/health_alerts`

### System Monitoring
- CPU, RAM, temperatura Raspberry Pi
- Status GPIO, sieci, USB/Serial
- Uptime i metryki systemowe

### Web Interface Dashboard
- Wizualizacja pozycji pojazdów w czasie rzeczywistym
- Wykresy pracy regulatora PID
- Monitoring zasobów systemu
- Kontrola parametrów i sterowanie

## Bezpieczeństwo

### Watchdog Serwa
- Timeout 0.2 sekundy
- Automatyczne ustawienie na 0° przy braku komend
- Tryb ręczny wyłącza watchdog

### Kontrola Sprzęgła
- Regulator wyłączany przy wciśniętym sprzęgle
- Reset całki PID przy dezaktywacji

### Monitoring Połączeń
- Sprawdzanie statusu Bluetooth
- Timeout danych GPS
- Alerty o problemach komunikacyjnych

## Rozwój i Testowanie

### Tryb Mockup
- Symulacja danych GPS bez sprzętu
- Testowanie logiki sterowania
- Rozwój interfejsu webowego

### Logowanie Danych
- Pakiet `data_logger` do nagrywania sesji
- Pliki .bag z danymi systemu
- Analiza post-processing

### Debugging
- Szczegółowe logi w każdym węźle
- Health reporting z metrykami
- Web interface z konsolą logów

---

*Dokumentacja systemu MSS - System synchronizacji prędkości i pozycji ciągnika rolniczego z sieczkarnią polową*
