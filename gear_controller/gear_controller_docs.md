# Gear Controller - Dokumentacja Pakietu

## Przegląd
Pakiet `gear_controller` odpowiada za sterowanie zmianą biegów ciągnika rolniczego. Węzeł odbiera komendy zmiany biegów i steruje przekaźnikami, które aktywują mechanizm zmiany biegów w ciągniku.

## Funkcjonalności
- **Sterowanie biegami**: Zmiana biegu w górę i w dół
- **Kontrola przekaźników**: Sterowanie pinami GPIO dla przekaźników
- **Health monitoring**: Raportowanie statusu węzła i GPIO
- **Bezpieczne sterowanie**: Kontrola czasu aktywacji przekaźników
- **Event publishing**: Publikowanie informacji o zmianach biegów

## Węzeł: `gear_shifter`

### Parametry
Pakiet nie definiuje własnych parametrów - używa stałych konfiguracji w kodzie.

### Konfiguracja GPIO
| Pin | Funkcja | Opis |
|-----|---------|------|
| 25 | Przekaźnik UP | Aktywacja zmiany biegu w górę |
| 20 | Przekaźnik DOWN | Aktywacja zmiany biegu w dół |

### Topiki

#### Publikowane
- **`/gears/events`** (`std_msgs/String`)
  - Informacje o zmianach biegów
  - Zawiera: 'UP' lub 'DOWN'
  - Publikowane przy każdej zmianie biegu

- **`/mss/node_health/gear_shifter`** (`std_msgs/String`)
  - Status zdrowia węzła w formacie JSON
  - Zawiera: status GPIO, konfigurację pinów, metryki systemu
  - Częstotliwość: 0.2 Hz (co 5s)

### Serwisy

#### `/gear_shift_up` (SetBool)
- **Funkcja**: Zmiana biegu w górę
- **Parametr**: `data` (bool) - zawsze true
- **Odpowiedź**: `success` (bool), `message` (string)

#### `/gear_shift_down` (SetBool)
- **Funkcja**: Zmiana biegu w dół
- **Parametr**: `data` (bool) - zawsze true
- **Odpowiedź**: `success` (bool), `message` (string)

## Architektura

### Algorytm sterowania
```python
def _trigger_relay(self, pin):
    # Aktywuj przekaźnik
    lgpio.gpio_write(self.chip_handle, pin, 1)  # Stan wysoki
    time.sleep(0.3)  # Czas aktywacji
    lgpio.gpio_write(self.chip_handle, pin, 0)  # Stan niski
```

### Logika zmiany biegów
1. **Otrzymanie komendy**: Serwis wywołuje callback
2. **Aktywacja przekaźnika**: Ustawienie pinu na HIGH
3. **Czas aktywacji**: 300ms (konfigurowalny)
4. **Dezaktywacja**: Ustawienie pinu na LOW
5. **Publikacja eventu**: Informacja o zmianie biegu

### Obsługa błędów
- **Błąd GPIO**: Logowanie ostrzeżeń i kontynuacja pracy
- **Błąd serwisu**: Zwracanie odpowiedzi z błędem
- **Graceful shutdown**: Prawidłowe zwalnianie zasobów GPIO

## Zależności

### ROS2
- `rclpy` - Python API dla ROS2
- `std_srvs` - Standardowe serwisy ROS2

### Python
- `lgpio` - Biblioteka GPIO dla Raspberry Pi 5
- `json` - Formatowanie danych health
- `psutil` - Metryki systemu
- `time` - Obsługa czasu i opóźnień

## Instalacja i uruchomienie

### Budowanie
```bash
cd /home/pi/mss_ros
colcon build --packages-select gear_controller
source install/setup.bash
```

### Uruchomienie
```bash
ros2 run gear_controller gear_shifter
```

### Uruchomienie z logami
```bash
ros2 run gear_controller gear_shifter --ros-args --log-level debug
```

## Konfiguracja sprzętowa

### Wymagania
- Raspberry Pi 5 (wymaga biblioteki lgpio)
- Przekaźniki do sterowania biegami
- Połączenia GPIO do przekaźników
- Zasilanie przekaźników (5V/12V)

### Schemat połączeń
```
Raspberry Pi 5 GPIO:
Pin 25 → Przekaźnik UP (sterowanie biegiem w górę)
Pin 20 → Przekaźnik DOWN (sterowanie biegiem w dół)
GND    → Wspólna masa przekaźników
VCC    → Zasilanie przekaźników (jeśli potrzebne)
```

### Konfiguracja uprawnień
```bash
# Sprawdź uprawnienia GPIO
ls -l /dev/gpiochip*

# Dodaj użytkownika do grupy gpio (jeśli potrzebne)
sudo usermod -a -G gpio $USER
```

## Diagnostyka

### Sprawdzanie statusu
```bash
# Sprawdź czy węzeł działa
ros2 node list | grep gear_shifter

# Sprawdź serwisy
ros2 service list | grep gear_shift

# Sprawdź topiki
ros2 topic list | grep gears

# Sprawdź health status
ros2 topic echo /mss/node_health/gear_shifter
```

### Testowanie
```bash
# Test zmiany biegu w górę
ros2 service call /gear_shift_up std_srvs/srv/SetBool "{data: true}"

# Test zmiany biegu w dół
ros2 service call /gear_shift_down std_srvs/srv/SetBool "{data: true}"

# Sprawdź eventy
ros2 topic echo /gears/events
```

### Testowanie GPIO
```bash
# Sprawdź status pinów GPIO
sudo lgpio-test 4 25  # Pin 25 (UP)
sudo lgpio-test 4 20  # Pin 20 (DOWN)
```

### Logi
```bash
# Sprawdź logi węzła
ros2 node info /gear_shifter

# Sprawdź parametry
ros2 param list /gear_shifter
```

### Typowe problemy
1. **Brak reakcji**: Sprawdź połączenia GPIO i zasilanie przekaźników
2. **Błąd GPIO**: Sprawdź uprawnienia i dostępność chipsetu
3. **Błąd serwisu**: Sprawdź czy węzeł jest aktywny
4. **Błąd biblioteki**: Upewnij się, że używasz Raspberry Pi 5 z lgpio

## Bezpieczeństwo

### GPIO
- Używaj odpowiednich przekaźników dla napięć ciągnika
- Sprawdź napięcia sygnałów (3.3V dla Raspberry Pi)
- Zabezpiecz przed zwarciami i przepięciami
- Używaj optoizolacji dla bezpieczeństwa

### Sterowanie
- Ogranicz czas aktywacji przekaźników (300ms)
- Implementuj cooldown między zmianami biegów
- Monitoruj częstotliwość zmian

## Wydajność

### Optymalizacja
- Czas aktywacji przekaźnika: 300ms (optymalny)
- Użyj odpowiedniego QoS dla topików
- Monitoruj wykorzystanie CPU i pamięci

### Metryki
- Czas reakcji serwisu: < 50ms
- Czas aktywacji przekaźnika: 300ms
- Wykorzystanie CPU: < 1%
- Wykorzystanie pamięci: < 20MB

## Testowanie

### Testy jednostkowe
```bash
# Uruchom testy
cd /home/pi/mss_ros
colcon test --packages-select gear_controller
colcon test-result --all
```

### Testy integracyjne
```bash
# Test z gear manager
ros2 run gear_manager gear_manager_node
ros2 service call /gear_manager/set_enabled std_srvs/srv/SetBool "{data: true}"

# Test ręcznego sterowania
ros2 service call /gear_shift_up std_srvs/srv/SetBool "{data: true}"
```

### Testy sprzętowe
```bash
# Test przekaźników
# 1. Wywołaj gear_shift_up - sprawdź czy przekaźnik UP się aktywuje
# 2. Wywołaj gear_shift_down - sprawdź czy przekaźnik DOWN się aktywuje
# 3. Sprawdź czas aktywacji (300ms)
# 4. Sprawdź czy przekaźniki się dezaktywują
```

## Graf przepływu informacji

```mermaid
graph TD
    A[Gear Manager] -->|/gear_shift_up| B[gear_shifter]
    C[Operator] -->|/gear_shift_down| B
    D[Manual Control] -->|SetBool| B
    
    B -->|GPIO 25| E[Przekaźnik UP]
    B -->|GPIO 20| F[Przekaźnik DOWN]
    B -->|/gears/events| G[System ROS2]
    B -->|/mss/node_health/gear_shifter| H[Health Monitor]
    
    subgraph "Wewnętrzne timery"
        I[Health Timer]
    end
    
    B --> I
    I -->|5s| H
    
    subgraph "GPIO Pins"
        J[Pin 25: UP Relay]
        K[Pin 20: DOWN Relay]
    end
    
    B --> J
    B --> K
    
    J -->|300ms pulse| E
    K -->|300ms pulse| F
    
    subgraph "Serwisy"
        L[/gear_shift_up]
        M[/gear_shift_down]
    end
    
    A --> L
    C --> M
    D --> L
    D --> M
    
    L --> B
    M --> B
```

## Kompatybilność

### Raspberry Pi
- **Raspberry Pi 5**: Pełna obsługa (lgpio)
- **Raspberry Pi 4**: Wymaga migracji na RPi.GPIO
- **Starsze wersje**: Wymaga migracji na RPi.GPIO

### Migracja z RPi.GPIO
```python
# Stary kod (RPi.GPIO)
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.OUT)
GPIO.output(pin, GPIO.HIGH)
time.sleep(0.3)
GPIO.output(pin, GPIO.LOW)

# Nowy kod (lgpio)
import lgpio
chip_handle = lgpio.gpiochip_open(4)
lgpio.gpio_claim_output(chip_handle, pin, 0)
lgpio.gpio_write(chip_handle, pin, 1)
time.sleep(0.3)
lgpio.gpio_write(chip_handle, pin, 0)
```

## Autorzy
- **Główny deweloper**: Adam Wróblewski
- **Email**: adam01wroblewski@gmail.com
