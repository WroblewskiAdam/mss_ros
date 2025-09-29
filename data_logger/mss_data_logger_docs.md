# MSS Data Logger - Dokumentacja

## Przegląd

Węzeł `mss_data_logger_node` to zaawansowany logger danych z całego systemu MSS (Maszyna Sieczkarnia System). Zbiera i zapisuje wszystkie kluczowe dane z systemu synchronizacji prędkości i pozycji ciągnika z sieczkarnią do pliku CSV.

## Funkcjonalności

- **Kompleksowe logowanie** - zbiera dane bezpośrednio z węzłów źródłowych
- **Obsługa placeholderów** - automatyczne zastępowanie brakujących danych
- **Synchronizacja czasowa** - wszystkie dane są zapisywane z tym samym znacznikiem czasu
- **Format CSV** - łatwe do analizy w Excel, Python, R
- **Automatyczne tworzenie plików** - z timestampem w nazwie
- **Wysoka częstotliwość** - 10 Hz (100 ms)

## Zbierane Dane

### Czas
- `ros_time_sec` - czas ROS (sekundy)
- `ros_time_nsec` - czas ROS (nanosekundy)
- `system_time` - czas systemowy Unix

### Pozycja Ciągnika
- `tractor_lat_raw` - szerokość geograficzna (surowe)
- `tractor_lon_raw` - długość geograficzna (surowe)
- `tractor_lat_filtered` - szerokość geograficzna (przefiltrowane)
- `tractor_lon_filtered` - długość geograficzna (przefiltrowane)

### Prędkość Ciągnika
- `tractor_speed_raw` - prędkość (surowe)
- `tractor_speed_filtered` - prędkość (przefiltrowane)

### Nawigacja Ciągnika
- `tractor_heading` - kurs (stopnie)
- `tractor_rtk_status` - status RTK (0-255)

### Biegi i Sprzęgło
- `tractor_gear` - aktualny półbieg (0-4)
- `tractor_clutch_state` - stan sprzęgła (0=zwolnione, 1=wciśnięte)

### Serwo
- `servo_position` - aktualna pozycja serwa (0-180°)

### Regulatory
- `speed_controller_enabled` - stan regulatora prędkości (0/1)
- `target_speed` - prędkość zadana (m/s)
- `position_controller_enabled` - stan regulatora pozycji (0/1)
- `target_position` - pozycja zadana (m)

### Pozycja Względna
- `distance_longitudinal` - odległość wzdłużna (m)
- `distance_lateral` - odległość poprzeczna (m)
- `distance_straight` - odległość w linii prostej (m)

### Sieczkarnia
- `chopper_lat_raw` - szerokość geograficzna (surowe)
- `chopper_lon_raw` - długość geograficzna (surowe)
- `chopper_lat_filtered` - szerokość geograficzna (przefiltrowane)
- `chopper_lon_filtered` - długość geograficzna (przefiltrowane)
- `chopper_speed_raw` - prędkość (surowe)
- `chopper_speed_filtered` - prędkość (przefiltrowane)
- `chopper_heading` - kurs (stopnie)
- `chopper_rtk_status` - status RTK (0-255)

### Status Systemu
- `bt_status` - status komunikacji Bluetooth (0/1)
- `autopilot_status` - status autopilota (tryb pracy)

## Architektura

### Subskrypcje
- `/gps_rtk_data/tractor_filtered` - dane GPS ciągnika (filtrowane)
- `/gps_rtk_data/chopper` - dane GPS sieczkarni
- `/servo/position` - pozycja serwa
- `/gears` - biegi i sprzęgło
- `/distance_metrics` - metryki odległości względnej
- `/speed_controller/state` - stan regulatora prędkości
- `/target_speed` - prędkość zadana
- `/target_position` - pozycja zadana
- `/autopilot/status` - status autopilota

### QoS
- **Reliability**: RELIABLE
- **Durability**: VOLATILE
- **Depth**: 10

### Placeholder Values
Gdy brakuje danych z węzłów, logger automatycznie zastępuje je wartościami placeholder:

- **PLACEHOLDER_UINT8**: `255` (dla statusów RTK, biegów, sprzęgła)
- **PLACEHOLDER_INT**: `99999` (dla pozycji serwa)
- **PLACEHOLDER_FLOAT**: `99999.0` (dla współrzędnych, prędkości, heading)

**Przykłady:**
- Brak danych z sieczkarni → `chopper_lat_raw = 99999.0`, `bt_status = 0`
- Brak danych z serwa → `servo_position = 99999`
- Brak danych z biegów → `tractor_gear = 255`

## Instalacja i Uruchomienie

### Budowanie
```bash
cd /home/pi/mss_ros
colcon build --packages-select data_logger
source install/setup.bash
```

### Uruchomienie
```bash
# Bezpośrednio
ros2 run data_logger mss_data_logger_node

# Przez launch file
ros2 launch data_logger mss_data_logger.launch.py
```

### Parametry
- `log_directory` - katalog do zapisu plików (domyślnie: ~/mss_logs)
- `log_frequency_hz` - częstotliwość zapisu (domyślnie: 10.0 Hz)

## Pliki Wyjściowe

### Lokalizacja
Pliki są zapisywane w katalogu `~/mss_logs/` z nazwą:
`mss_system_log_YYYYMMDD_HHMMSS.csv`

### Przykład
```
/home/pi/mss_logs/mss_system_log_20241201_143022.csv
```

### Format CSV
Plik zawiera nagłówki w pierwszym wierszu i dane w kolejnych wierszach, oddzielone przecinkami.

## Monitorowanie

### Logi
Węzeł loguje informacje o:
- Inicjalizacji
- Liczbie zapisanych rekordów (co 100)
- Zamykaniu pliku

### Status
- Sprawdź czy węzeł działa: `ros2 node list | grep mss_data_logger`
- Sprawdź topiki: `ros2 topic list | grep diagnostics`

## Integracja z Systemem

### Launch Files
Dodaj do głównego launch file systemu:
```python
Node(
    package='data_logger',
    executable='mss_data_logger_node',
    name='mss_data_logger_node',
    output='screen',
    emulate_tty=True,
)
```

### Zależności
Węzeł wymaga uruchomionych:
- `diagnostics_node` - główny strumień danych
- `relative_computer_node` - metryki odległości
- `speed_controller_node` - stan regulatora
- `mss_health_monitor_node` - health systemu

## Analiza Danych

### Python
```python
import pandas as pd
import matplotlib.pyplot as plt

# Wczytaj dane
df = pd.read_csv('mss_system_log_20241201_143022.csv')

# Analiza prędkości
plt.plot(df['ros_time_sec'], df['tractor_speed_filtered'])
plt.xlabel('Czas (s)')
plt.ylabel('Prędkość (m/s)')
plt.title('Prędkość Ciągnika')
plt.show()
```

### Excel
1. Otwórz plik CSV w Excel
2. Użyj funkcji filtrowania
3. Stwórz wykresy z danych

## Troubleshooting

### Problem: Brak danych
```bash
# Sprawdź czy węzeł działa
ros2 node list | grep mss_data_logger

# Sprawdź topiki
ros2 topic echo /diagnostics --once
```

### Problem: Pusty plik
- Sprawdź czy `diagnostics_node` publikuje dane
- Sprawdź uprawnienia do zapisu w katalogu

### Problem: Błędy QoS
- Upewnij się że wszystkie węzły używają RELIABLE QoS
- Sprawdź czy topiki są publikowane

## Wydajność

### Zużycie Zasobów
- **CPU**: ~2-5%
- **RAM**: ~50MB
- **Dysk**: ~1MB/min (przy 10 Hz)

### Optymalizacja
- Dostosuj częstotliwość zapisu
- Użyj SSD dla lepszej wydajności
- Regularnie archiwizuj stare pliki

## Bezpieczeństwo

### Pliki
- Pliki są zapisywane lokalnie
- Brak transmisji danych przez sieć
- Automatyczne zamykanie przy wyłączeniu

### Dane
- Zawierają dokładne pozycje GPS
- Nie udostępniaj publicznie
- Szyfruj jeśli konieczne

## Autorzy
- **Główny deweloper**: Adam Wróblewski
- **Email**: adam01wroblewski@gmail.com
