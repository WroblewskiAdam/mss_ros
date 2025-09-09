# 🎭 SYSTEM MOCKUP - MSS GPS SIMULATION

## Przegląd

System mockup umożliwia testowanie i rozwój systemu MSS bez potrzeby posiadania prawdziwego sprzętu GPS RTK i komunikacji Bluetooth z sieczkarnią.

## 🚀 Szybki Start

### 1. Budowanie pakietu mockup
```bash
cd /home/pi/mss_ros
colcon build --packages-select gps_mockup
source install/setup.bash
```

### 2. Test podstawowy
```bash
cd /home/pi/mss_ros/src
./test_mockup.sh
```

### 3. Uruchomienie pełnego systemu z mockup
```bash
cd /home/pi/mss_ros/src
./mss_startup_mockup.sh
```

## 📁 Struktura Plików

```
gps_mockup/
├── gps_mockup/
│   └── mockup_node.py          # Główny węzeł symulacji
├── package.xml                 # Konfiguracja pakietu
├── setup.py                    # Setup Python
└── README.md                   # Dokumentacja pakietu

mss_bringup/launch/
└── all_nodes_mockup.launch.py  # Launch file z mockup

mss_startup_mockup.sh           # Skrypt uruchamiający
test_mockup.sh                  # Skrypt testowy
```

## 🎯 Funkcjonalności Mockup

### Symulacja Ruchu
- **Ciągnik**: porusza się z prędkością 2 m/s (~7.2 km/h)
- **Sieczkarnia**: jedzie 5m za ciągnikiem
- **Realistyczne zakręty**: sinusoidalne odchylenia symulujące nierówności terenu
- **Kierunek**: głównie na północ z lekkimi zakrętami

### Dane GPS RTK
- **Współrzędne**: Warszawa jako punkt bazowy (52.2297, 21.0122)
- **Kurs**: realistyczne zmiany kierunku
- **Prędkość**: stała z niewielkimi wahaniami
- **Wysokość**: 120m ±2m
- **Status RTK**: rotacja między SPS, DGPS, FIX, FLOAT
- **Czas GPS**: aktualny czas systemowy

### Topiki Publikowane
- `/gps_rtk_data_filtered` - dane GPS ciągnika
- `/gps_rtk_data/chopper` - dane GPS sieczkarni

## ⚙️ Konfiguracja

### Parametry ROS
```bash
# Zmiana prędkości ciągnika
ros2 param set /gps_mockup_node tractor_speed_mps 3.0

# Zmiana odległości sieczkarni
ros2 param set /gps_mockup_node chopper_offset_m 7.0

# Zmiana częstotliwości publikacji
ros2 param set /gps_mockup_node publish_frequency_hz 20.0
```

### Parametry w Launch File
```python
parameters=[{
    'publish_frequency_hz': 10.0,
    'tractor_speed_mps': 2.0,
    'chopper_offset_m': 5.0,
    'simulation_area_lat': 52.2297,
    'simulation_area_lon': 21.0122,
}]
```

## 🔄 Różnice względem Prawdziwego Systemu

### Wyłączone w Mockup:
- `gps_rtk_node` - prawdziwy odczyt GPS RTK
- `bt_receiver_node` - komunikacja Bluetooth z sieczkarnią

### Aktywne w Mockup:
- `gps_mockup_node` - symulacja danych GPS
- Wszystkie węzły sterowania i monitoringu
- Web interface z pełną funkcjonalnością

## 🧪 Testowanie

### Sprawdzenie Topików
```bash
# Lista topików
ros2 topic list | grep gps

# Dane ciągnika
ros2 topic echo /gps_rtk_data_filtered

# Dane sieczkarni  
ros2 topic echo /gps_rtk_data/chopper

# Częstotliwość
ros2 topic hz /gps_rtk_data_filtered
```

### Sprawdzenie Węzłów
```bash
# Lista węzłów
ros2 node list

# Informacje o węźle
ros2 node info /gps_mockup_node
```

### Web Interface
Po uruchomieniu systemu z mockup:
1. Uruchom web interface: `cd operator_interface/web && ./start_interface.sh`
2. Otwórz: `http://localhost:8080`
3. Sprawdź zakładki:
   - **Główny**: wizualizacja pozycji pojazdów
   - **Szczegóły**: dane GPS w czasie rzeczywistym
   - **System**: monitoring zasobów
   - **Health**: status węzłów

## 🐛 Troubleshooting

### Problem: Węzeł się nie uruchamia
```bash
# Sprawdź budowanie
colcon build --packages-select gps_mockup

# Sprawdź zależności
rosdep install --from-paths src --ignore-src -r -y
```

### Problem: Brak danych
```bash
# Sprawdź czy węzeł jest aktywny
ros2 node list | grep mockup

# Sprawdź logi
ros2 node info /gps_mockup_node
```

### Problem: Web interface nie działa
```bash
# Sprawdź ROS Bridge
ros2 topic list | grep rosbridge

# Sprawdź węzeł diagnostics
ros2 node list | grep diagnostics
```

## 📊 Monitoring

### Logi Węzła
```bash
# Logi w czasie rzeczywistym
ros2 topic echo /rosout | grep mockup

# Informacje o węźle
ros2 node info /gps_mockup_node
```

### Metryki Wydajności
```bash
# Częstotliwość publikacji
ros2 topic hz /gps_rtk_data_filtered

# Opóźnienie
ros2 topic delay /gps_rtk_data_filtered
```

## 🔧 Rozwój

### Dodanie Nowych Funkcji
1. Edytuj `mockup_node.py`
2. Dodaj nowe parametry w `package.xml`
3. Zbuduj pakiet: `colcon build --packages-select gps_mockup`
4. Przetestuj: `./test_mockup.sh`

### Integracja z Innymi Węzłami
Mockup jest w pełni kompatybilny z istniejącymi węzłami:
- `diagnostics_node` - agreguje dane mockup
- `relative_computer_node` - oblicza pozycję względną
- `speed_controller_node` - steruje prędkością
- `gear_manager_node` - zarządza biegami

## 📝 Notatki

- Mockup symuluje realistyczne warunki pracy
- Dane są publikowane z częstotliwością 10 Hz
- System jest w pełni funkcjonalny bez prawdziwego sprzętu
- Idealny do testowania i rozwoju oprogramowania
- Można łatwo dostosować parametry symulacji
