# PLAN IMPLEMENTACYJNY - UJEDNOLICENIE SYSTEMU MSS
## Cel: Automatyczne uruchamianie + Web App jako Control Center
## Czas realizacji: 1 dzień

---

## 🚀 ETAP 1: SYSTEMD SERVICES (2 godziny)

### 1.1 Stworzenie plików systemd
**Lokalizacja:** `/etc/systemd/system/`

#### `mss-master.service`
```ini
[Unit]
Description=MSS ROS2 Master Service
After=network.target
Wants=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/mss_ros/src
ExecStart=/opt/ros/humble/bin/ros2 launch mss_bringup all_nodes.launch.py
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

#### `mss-watchdog.service`
```ini
[Unit]
Description=MSS System Watchdog
After=mss-master.service
Requires=mss-master.service

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/mss_ros/src
ExecStart=/home/pi/mss_ros/src/mss_watchdog.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

### 1.2 Skrypt startup
**Lokalizacja:** `/home/pi/mss_ros/src/mss_startup.sh`
- Sprawdzanie sprzętu
- Inicjalizacja ROS
- Sekwencyjne uruchamianie

---

## 🎮 ETAP 2: WATCHDOG SYSTEMOWY (2 godziny)

### 2.1 Stworzenie węzła watchdog
**Lokalizacja:** `mss_watchdog/mss_watchdog/mss_watchdog_node.py`

**Funkcje:**
- Monitorowanie statusu wszystkich węzłów
- Automatyczny restart zawieszonych węzłów
- Health checks komunikacji ROS
- Monitoring zasobów systemu

**Topiki:**
- `/mss/system_status` - ogólny status systemu
- `/mss/node_status` - status poszczególnych węzłów
- `/mss/health_alerts` - alerty o problemach

**Serwisy:**
- `/mss/restart_node` - restart konkretnego węzła
- `/mss/system_control` - start/stop całego systemu

### 2.2 Dodanie do launch file
**Modyfikacja:** `mss_bringup/launch/all_nodes.launch.py`
```python
# Dodanie węzła watchdog
Node(
    package='mss_watchdog',
    executable='mss_watchdog_node',
    name='mss_watchdog_node',
    output='screen',
    emulate_tty=True,
),
```

---

## 🌐 ETAP 3: ROZSZERZENIE WEB APP (4 godziny)

### 3.1 Nowe pliki JavaScript
**Lokalizacja:** `operator_interface/web/`

#### `system_control.js` - Zarządzanie systemem
- Start/stop/restart węzłów
- Status wszystkich węzłów
- Kontrola systemu

#### `system_monitor.js` - Monitoring systemu
- CPU, RAM, temperatura RPi
- Logi w czasie rzeczywistym
- Alerty i powiadomienia

#### `config_manager.js` - Zarządzanie konfiguracją
- Edycja parametrów ROS
- Profile pracy
- Backup/restore

### 3.2 Modyfikacja `main.js`
**Dodanie:**
- Inicjalizacja nowych modułów
- Subskrypcje na topiki systemowe
- Obsługa nowych funkcji

### 3.3 Modyfikacja `index.html`
**Dodanie:**
- Nowe zakładki: System, Monitoring, Konfiguracja
- Dashboard statusu węzłów
- Panel kontrolny systemu

### 3.4 Modyfikacja `style.css`
**Dodanie:**
- Style dla nowych elementów
- Responsywny design
- Ciemny motyw

---

## 🔧 ETAP 4: MODYFIKACJE WĘZŁÓW ROS (2 godziny)

### 4.1 Dodanie health reporting do wszystkich węzłów
**Modyfikacja każdego węzła:**

#### `speed_controller_node.py`
```python
# Dodanie publisher'a statusu
self.health_pub = self.create_publisher(
    String, '/mss/node_health/speed_controller', 10
)

# Dodanie timer'a health check
self.health_timer = self.create_timer(5.0, self.publish_health)

def publish_health(self):
    health_msg = String()
    health_msg.data = json.dumps({
        'status': 'running',
        'last_update': time.time(),
        'errors': [],
        'warnings': []
    })
    self.health_pub.publish(health_msg)
```

#### `servo_controller.py`
```python
# Podobne modyfikacje
self.health_pub = self.create_publisher(
    String, '/mss/node_health/servo_controller', 10
)
```

#### `gps_rtk_node.py`
```python
# Podobne modyfikacje
self.health_pub = self.create_publisher(
    String, '/mss/node_health/gps_rtk', 10
)
```

### 4.2 Dodanie serwisów kontrolnych
**Każdy węzeł dostaje:**
- `/node/restart` - restart węzła
- `/node/status` - status węzła
- `/node/configure` - zmiana parametrów

---

## 📊 ETAP 5: NOWE TOPIKI I WIADOMOŚCI (1 godzina)

### 5.1 Nowe wiadomości
**Lokalizacja:** `my_robot_interfaces/msg/`

#### `SystemStatus.msg`
```
std_msgs/Header header
string overall_status  # OK, WARNING, ERROR
NodeStatus[] nodes
SystemMetrics metrics
```

#### `NodeStatus.msg`
```
string node_name
string status  # RUNNING, STOPPED, ERROR, RESTARTING
string[] errors
string[] warnings
time last_heartbeat
```

#### `SystemMetrics.msg`
```
float64 cpu_usage
float64 memory_usage
float64 temperature
float64 disk_usage
```

### 5.2 Nowe serwisy
**Lokalizacja:** `my_robot_interfaces/srv/`

#### `SystemControl.srv`
```
bool enable
---
bool success
string message
```

#### `NodeControl.srv`
```
string node_name
string action  # START, STOP, RESTART
---
bool success
string message
```

---

## ⚙️ ETAP 6: KONFIGURACJA I AUTOSTART (1 godzina)

### 6.1 Plik konfiguracyjny
**Lokalizacja:** `/home/pi/mss_ros/src/mss_config.yaml`
```yaml
system:
  autostart: true
  watchdog_enabled: true
  health_check_interval: 5.0
  
nodes:
  gps_rtk:
    priority: 1
    restart_on_failure: true
    max_restart_attempts: 3
    
  speed_controller:
    priority: 2
    restart_on_failure: true
    max_restart_attempts: 3
    
  servo_controller:
    priority: 3
    restart_on_failure: true
    max_restart_attempts: 3

profiles:
  harvest:
    name: "Tryb zbiorów"
    description: "Pełna automatyzacja"
    enabled_nodes: ["gps_rtk", "speed_controller", "servo_controller", "gear_manager"]
    
  transport:
    name: "Tryb transportu"
    description: "Podstawowe funkcje"
    enabled_nodes: ["gps_rtk", "speed_controller"]
```

### 6.2 Skrypt instalacyjny
**Lokalizacja:** `/home/pi/mss_ros/src/install_mss_system.sh`
```bash
#!/bin/bash
# Instalacja systemu MSS jako usługi systemd

echo "Instalacja systemu MSS..."

# Kopiowanie plików systemd
sudo cp mss-master.service /etc/systemd/system/
sudo cp mss-watchdog.service /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Włączenie autostartu
sudo systemctl enable mss-master.service
sudo systemctl enable mss-watchdog.service

echo "Instalacja zakończona!"
echo "Uruchomienie: sudo systemctl start mss-master.service"
```

---

## 🎯 ETAP 7: TESTING I DEBUGGING (2 godziny)

### 7.1 Testy funkcjonalne
- Uruchomienie systemu przez systemd
- Test watchdog'a
- Test web app
- Test restart'ów węzłów

### 7.2 Debugging
- Naprawa błędów
- Optymalizacja wydajności
- Testowanie edge cases

---

## 📋 LISTA ZADAŃ DO WYKONANIA

### Dzień 1 - Rano (4 godziny)
- [ ] ETAP 1: Systemd services
- [ ] ETAP 2: Watchdog systemowy
- [ ] ETAP 4: Modyfikacje węzłów ROS

### Dzień 1 - Popołudnie (4 godziny)
- [ ] ETAP 3: Rozszerzenie web app
- [ ] ETAP 5: Nowe topiki i wiadomości
- [ ] ETAP 6: Konfiguracja i autostart
- [ ] ETAP 7: Testing i debugging

---

## 🔍 KONTROLA JAKOŚCI

### Przed zakończeniem sprawdzić:
- [ ] System uruchamia się automatycznie po reboot
- [ ] Web app pokazuje status wszystkich węzłów
- [ ] Watchdog automatycznie restartuje zawieszone węzły
- [ ] Można zarządzać systemem z poziomu web app
- [ ] Wszystkie funkcje działają poprawnie
- [ ] System jest stabilny i odporny na błędy

---

## 📝 NOTATKI IMPLEMENTACYJNE

### Ważne uwagi:
1. **Backup** - przed rozpoczęciem zrobić backup całego projektu
2. **Testowanie** - każdy etap testować przed przejściem dalej
3. **Logi** - dodać szczegółowe logowanie do debugowania
4. **Błędy** - przygotować fallback na wypadek problemów

### Pliki do modyfikacji:
- `mss_bringup/launch/all_nodes.launch.py`
- `operator_interface/web/main.js`
- `operator_interface/web/index.html`
- `operator_interface/web/style.css`
- Wszystkie węzły ROS (dodanie health reporting)

### Nowe pliki do stworzenia:
- `mss_watchdog/` - cały pakiet
- `mss_config.yaml`
- `install_mss_system.sh`
- Pliki systemd
- Nowe wiadomości i serwisy
