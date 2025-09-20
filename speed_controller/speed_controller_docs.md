# Speed Controller - Dokumentacja Pakietu

## Przegląd
Pakiet `speed_controller` zawiera zaawansowany system regulacji prędkości ciągnika rolniczego z algorytmem PID i feedforward. Pakiet składa się z trzech węzłów: filtra prędkości, głównego regulatora PID oraz węzła teleop do ręcznego sterowania.

## Funkcjonalności
- **Filtrowanie prędkości**: Filtr Butterworth do usuwania szumów GPS
- **Regulacja PID**: Zaawansowany regulator z feedforward i anti-windup
- **Sterowanie serwem**: Komendy kąta serwa na podstawie błędu prędkości
- **Feedforward**: Wielomiany 3. stopnia dla inicjalizacji integratora
- **Health monitoring**: Raportowanie statusu wszystkich węzłów
- **Teleop**: Ręczne sterowanie prędkością przez klawiaturę

## Węzły

### 1. `speed_filter_node`
Filtr Butterworth do wygładzania danych prędkości z GPS.

#### Parametry
| Parametr | Typ | Domyślna wartość | Opis |
|----------|-----|------------------|------|
| `filter_cutoff_hz` | double | `0.8` | Częstotliwość odcięcia filtru [Hz] |
| `filter_order` | int | `2` | Rząd filtru Butterworth |
| `sampling_frequency_hz` | double | `20.0` | Częstotliwość próbkowania [Hz] |

#### Topiki
- **Subskrypcje**: `/gps_rtk_data` (GpsRtk)
- **Publikacje**: `/gps_rtk_data_filtered` (GpsRtk), `/mss/node_health/speed_filter_node` (String)

### 2. `speed_controller_node`
Główny regulator prędkości PID z feedforward.

#### Parametry
| Parametr | Typ | Domyślna wartość | Opis |
|----------|-----|------------------|------|
| `kp` | double | `10.0` | Wzmocnienie proporcjonalne |
| `ki` | double | `20.0` | Wzmocnienie całkowe |
| `kd` | double | `0.0` | Wzmocnienie różniczkowe |
| `v_idle` | double | `1.3359` | Prędkość jałowa [m/s] |
| `servo_min_angle` | int | `0` | Minimalny kąt serwa [°] |
| `servo_max_angle` | int | `150` | Maksymalny kąt serwa [°] |
| `controller_frequency` | double | `20.0` | Częstotliwość regulatora [Hz] |
| `output_min` | double | `0.0` | Minimalne wyjście regulatora |
| `output_max` | double | `150.0` | Maksymalne wyjście regulatora |

#### Topiki
- **Subskrypcje**: 
  - `/target_speed` (Float64) - Prędkość zadana
  - `/gps_rtk_data_filtered` (GpsRtk) - Filtrowana prędkość
  - `/gears` (Gear) - Stan biegów i sprzęgła
- **Publikacje**:
  - `/servo/set_angle` (StampedInt32) - Komenda kąta serwa
  - `/speed_controller/state` (SpeedControllerState) - Stan regulatora
  - `/mss/node_health/speed_controller_node` (String) - Health status

#### Serwisy
- **`/speed_controller/set_enabled`** (SetBool) - Włączanie/wyłączanie autopilota
- **`/speed_controller_node/set_parameters`** (SetParameters) - Ustawianie parametrów PID

### 3. `speed_teleop_node`
Węzeł do ręcznego sterowania prędkością przez klawiaturę.

#### Funkcjonalności
- Sterowanie strzałkami: ↑/↓ (zmiana prędkości), ←/→ (zmiana biegów)
- Automatyczne publikowanie prędkości zadanej
- Interfejs tekstowy z aktualnym stanem

## Algorytm Regulatora PID

### Równanie PID
```
u(t) = Kp * e(t) + Ki * ∫e(τ)dτ + Kd * de(t)/dt
```

### Anti-windup
Integrator jest zatrzymywany gdy:
- Wyjście osiąga maksimum i błąd > 0
- Wyjście osiąga minimum i błąd < 0

### Feedforward
Wielomiany 3. stopnia dla inicjalizacji integratora:
```python
angle = a3*s³ + a2*s² + a1*s + a0
```
gdzie `s` to prędkość względem prędkości jałowej.

### Współczynniki Feedforward
| Bieg | a3 | a2 | a1 | a0 |
|------|----|----|----|----|
| 1 | 0.5874 | 13.4004 | 14.6534 | -34.5274 |
| 2 | 1.4601 | -0.7756 | 38.1844 | -57.7960 |
| 3 | 3.8219 | -22.3852 | 85.7275 | -99.1968 |
| 4 | 0.3258 | 0.5479 | 17.6930 | -45.4661 |

## Wiadomości

### `SpeedControllerState.msg`
```yaml
std_msgs/Header header
float64 setpoint_speed    # Prędkość zadana [m/s]
float64 current_speed     # Aktualna prędkość [m/s]
float64 control_output    # Wyjście regulatora [°]
```

## Architektura

### Przepływ danych
1. **GPS** → `/gps_rtk_data` → **Speed Filter** → `/gps_rtk_data_filtered`
2. **Speed Controller** ← `/gps_rtk_data_filtered` + `/target_speed` + `/gears`
3. **Speed Controller** → `/servo/set_angle` → **Servo Controller**

### Logika sterowania
```python
if autopilot_enabled and not clutch_pressed:
    # Oblicz błąd prędkości
    error = (target_speed - v_idle) - (current_speed - v_idle)
    
    # Oblicz wyjście PID
    output_p = Kp * error
    output_i = Ki * error * dt
    output_d = Kd * (error - prev_error) / dt
    
    # Anti-windup
    if not ((output >= max_output and error > 0) or 
            (output <= min_output and error < 0)):
        integral_sum += output_i
    
    # Ograniczenie wyjścia
    output = max(min_output, min(max_output, output_p + integral_sum))
else:
    # Wyłącz regulator
    integral_sum = 0.0
    output = servo_min_angle
```

## Zależności

### ROS2
- `rclpy` - Python API dla ROS2
- `std_msgs` - Standardowe wiadomości
- `my_robot_interfaces` - Niestandardowe wiadomości
- `gps_rtk_msgs` - Wiadomości GPS

### Python
- `scipy.signal` - Filtr Butterworth
- `numpy` - Obliczenia numeryczne
- `json` - Formatowanie danych health
- `psutil` - Metryki systemu
- `time` - Obsługa czasu

## Instalacja i uruchomienie

### Budowanie
```bash
cd /home/pi/mss_ros
colcon build --packages-select speed_controller
source install/setup.bash
```

### Uruchomienie wszystkich węzłów
```bash
# Filtr prędkości
ros2 run speed_controller speed_filter_node

# Regulator PID
ros2 run speed_controller speed_controller_node

# Teleop (opcjonalnie)
ros2 run speed_controller speed_teleop_node
```

### Uruchomienie z parametrami
```bash
ros2 run speed_controller speed_controller_node --ros-args \
  -p kp:=15.0 \
  -p ki:=25.0 \
  -p kd:=1.0 \
  -p controller_frequency:=30.0
```

## Konfiguracja

### Tuning parametrów PID
```bash
# Ustawienie parametrów przez serwis
ros2 service call /speed_controller_node/set_parameters \
  rcl_interfaces/srv/SetParameters \
  "{parameters: [{name: 'kp', value: {double_value: 12.0}}, 
                 {name: 'ki', value: {double_value: 22.0}}]}"

# Sprawdzenie parametrów
ros2 param get /speed_controller_node kp
ros2 param get /speed_controller_node ki
```

### Konfiguracja filtra
```bash
# Ustawienie częstotliwości odcięcia
ros2 param set /speed_filter_node filter_cutoff_hz 1.0

# Ustawienie rzędu filtru
ros2 param set /speed_filter_node filter_order 3
```

## Diagnostyka

### Sprawdzanie statusu
```bash
# Sprawdź węzły
ros2 node list | grep speed

# Sprawdź topiki
ros2 topic list | grep speed

# Sprawdź dane regulatora
ros2 topic echo /speed_controller/state

# Sprawdź health status
ros2 topic echo /mss/node_health/speed_controller_node
```

### Monitoring wydajności
```bash
# Sprawdź częstotliwość publikacji
ros2 topic hz /speed_controller/state

# Sprawdź opóźnienie
ros2 topic delay /speed_controller/state

# Sprawdź parametry
ros2 param list /speed_controller_node
```

### Testowanie
```bash
# Test ręcznego sterowania
ros2 topic pub /target_speed std_msgs/msg/Float64 "{data: 2.5}"

# Test włączania autopilota
ros2 service call /speed_controller/set_enabled std_srvs/srv/SetBool "{data: true}"

# Test teleop
ros2 run speed_controller speed_teleop_node
```

## Bezpieczeństwo

### Warunki bezpieczeństwa
- **Sprzęgło wciśnięte**: Regulator wyłączony
- **Autopilot wyłączony**: Serwo w pozycji 0°
- **Brak danych GPS**: Regulator zatrzymany
- **Watchdog**: Automatyczne wyłączenie przy braku komend

### Ograniczenia
- Kąt serwa: 0-150°
- Prędkość zadana: ≥ v_idle
- Częstotliwość regulatora: 20 Hz
- Timeout danych: 2s

## Wydajność

### Metryki
- Częstotliwość regulatora: 20 Hz
- Opóźnienie: < 50ms
- Wykorzystanie CPU: < 5%
- Wykorzystanie pamięci: < 50MB

### Optymalizacja
- Użyj odpowiedniego QoS dla topików
- Dostosuj częstotliwość regulatora do potrzeb
- Monitoruj wykorzystanie zasobów

## Testowanie

### Testy jednostkowe
```bash
# Uruchom testy
cd /home/pi/mss_ros
colcon test --packages-select speed_controller
colcon test-result --all
```

### Testy integracyjne
```bash
# Test z mockup GPS
ros2 run system_mockup gps_mockup_node

# Test regulatora
ros2 topic pub /target_speed std_msgs/msg/Float64 "{data: 2.0}"
ros2 service call /speed_controller/set_enabled std_srvs/srv/SetBool "{data: true}"
```

### Testy wydajności
```bash
# Test częstotliwości
ros2 topic hz /speed_controller/state

# Test opóźnienia
ros2 topic delay /speed_controller/state
```

## Graf przepływu informacji

```mermaid
graph TD
    A[GPS RTK] -->|/gps_rtk_data| B[speed_filter_node]
    B -->|/gps_rtk_data_filtered| C[speed_controller_node]
    D[Operator] -->|/target_speed| C
    E[Gear Reader] -->|/gears| C
    C -->|/servo/set_angle| F[Servo Controller]
    C -->|/speed_controller/state| G[System ROS2]
    C -->|/mss/node_health/speed_controller_node| H[Health Monitor]
    
    subgraph "Serwisy"
        I[/speed_controller/set_enabled]
        J[/speed_controller_node/set_parameters]
    end
    
    K[Operator] -->|SetBool| I
    K -->|SetParameters| J
    I --> C
    J --> C
    
    subgraph "Teleop"
        L[speed_teleop_node]
        M[Keyboard Input]
    end
    
    M --> L
    L -->|/target_speed| C
    
    subgraph "Algorytm PID"
        N[Error Calculation]
        O[PID Controller]
        P[Anti-windup]
        Q[Feedforward]
    end
    
    C --> N
    N --> O
    O --> P
    P --> Q
    Q --> F
```

## Autorzy
- **Główny deweloper**: Adam Wróblewski
- **Email**: adam01wroblewski@gmail.com
