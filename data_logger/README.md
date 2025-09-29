# Data Logger Package

Pakiet do logowania danych z systemu MSS (Maszyna Sieczkarnia System) do plików CSV.

## Węzły

### 1. `logger_node` (oryginalny)
- Loguje dane z IMU, GPS i serwa
- Zapisuje do oddzielnych plików CSV
- Kompatybilny z istniejącym systemem

### 2. `mss_data_logger_node` (nowy)
- **Główny logger systemu MSS**
- Zbiera wszystkie dane z systemu synchronizacji
- Jeden plik CSV z wszystkimi danymi
- Częstotliwość: 10 Hz

### 3. `test_mss_logger`
- Skrypt testowy sprawdzający dostępność topików
- Pomocny w diagnostyce systemu

## Uruchomienie

### MSS Data Logger
```bash
# Bezpośrednio
ros2 run data_logger mss_data_logger_node

# Przez launch file
ros2 launch data_logger mss_data_logger.launch.py
```

### Test topików
```bash
ros2 run data_logger test_mss_logger
```

## Pliki wyjściowe

### MSS Data Logger
- **Lokalizacja**: `~/mss_logs/`
- **Format**: `mss_system_log_YYYYMMDD_HHMMSS.csv`
- **Zawartość**: Wszystkie dane z systemu MSS

### Oryginalny Logger
- **Lokalizacja**: `~/ros2_logs_raw/`
- **Formaty**: 
  - `gps_servo_log_YYYYMMDD_HHMMSS.csv`
  - `imu_log_YYYYMMDD_HHMMSS.csv`
  - `speed_control_log_YYYYMMDD_HHMMSS.csv`

## Wymagane topiki

### MSS Data Logger
- `/diagnostics` - główny strumień danych
- `/distance_metrics` - metryki odległości
- `/speed_controller/state` - stan regulatora
- `/target_speed` - prędkość zadana
- `/target_position` - pozycja zadana
- `/mss/system_health` - health systemu

## Dokumentacja

Szczegółowa dokumentacja: [mss_data_logger_docs.md](mss_data_logger_docs.md)

## Autorzy
- Adam Wróblewski (adam01wroblewski@gmail.com)
