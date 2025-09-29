# MQTT Communication Package - Dokumentacja

## Przegląd

Pakiet `mqtt_comm` odpowiada za komunikację MQTT z sieczkarnią, zastępując niestabilną komunikację Bluetooth. Wykorzystuje istniejące połączenie internetowe do GPS RTK dla większej stabilności i zasięgu.

## Funkcjonalności

- **Komunikacja MQTT** - odbiór danych GPS z sieczkarni przez MQTT
- **Automatyczne ponowne połączenie** - w przypadku utraty połączenia
- **Walidacja danych** - sprawdzanie poprawności danych GPS
- **Health monitoring** - raportowanie statusu połączenia
- **Konfigurowalne parametry** - broker, topic, uwierzytelnianie
- **Kompatybilność** - publikuje na tym samym topiku co Bluetooth

## Węzeł: `mqtt_chopper_receiver_node`

### Parametry

| Parametr | Typ | Domyślna wartość | Opis |
|----------|-----|------------------|------|
| `mqtt_broker_host` | string | `mss-mqtt.ddns.net` | Adres brokera MQTT |
| `mqtt_broker_port` | int | `1883` | Port brokera MQTT |
| `mqtt_topic_chopper` | string | `test/polaczenia` | Topic dla danych GPS sieczkarni |
| `mqtt_keepalive` | int | `60` | Keepalive MQTT [s] |
| `mqtt_username` | string | `` | Nazwa użytkownika MQTT |
| `mqtt_password` | string | `` | Hasło MQTT |
| `reconnect_interval` | double | `5.0` | Interwał ponownego połączenia [s] |

### Topiki

#### Publikowane
- **`/gps_rtk_data/chopper`** (`my_robot_interfaces/GpsRtk`)
  - Dane GPS sieczkarni z MQTT
  - Zawiera: pozycję, prędkość, kurs, status RTK
  - Częstotliwość: zależna od danych z sieczkarni

- **`/mss/node_health/mqtt_chopper_receiver`** (`std_msgs/String`)
  - Status zdrowia węzła w formacie JSON
  - Zawiera: status połączenia, statystyki, metryki
  - Częstotliwość: 0.2 Hz (co 5s)

## Architektura

### Format danych MQTT

Węzeł oczekuje wiadomości JSON w formacie sendera:
```json
{
    "lat": 52.123456,
    "lon": 21.123456,
    "speed": 2.5,
    "heading": 45.0,
    "rtk_status": 4,
    "gps_time": "123456.78"
}
```

### Algorytm działania
```python
def on_mqtt_message(self, client, userdata, msg):
    # 1. Dekoduj JSON
    message_data = json.loads(msg.payload.decode('utf-8'))
    
    # 2. Waliduj dane
    if self.validate_gps_data(message_data):
        # 3. Konwertuj na ROS2
        gps_msg = self.convert_to_gps_rtk(message_data)
        
        # 4. Opublikuj
        self.gps_publisher.publish(gps_msg)
```

### Obsługa błędów
- **Błąd połączenia** → automatyczne ponowne połączenie
- **Błąd parsowania JSON** → logowanie błędu, kontynuacja
- **Nieprawidłowe dane** → walidacja, odrzucenie wiadomości
- **Timeout** → ponowne połączenie z brokerem

## Instalacja i Uruchomienie

### Budowanie
```bash
cd /home/pi/mss_ros
colcon build --packages-select mqtt_comm
source install/setup.bash
```

### Uruchomienie
```bash
# Bezpośrednio
ros2 run mqtt_comm mqtt_chopper_receiver_node

# Przez launch file
ros2 launch mqtt_comm mqtt_chopper_receiver.launch.py
```

### Uruchomienie z parametrami
```bash
ros2 run mqtt_comm mqtt_chopper_receiver_node --ros-args \
  -p mqtt_broker_host:=mss-mqtt.ddns.net \
  -p mqtt_topic_chopper:=chopper/gps \
  -p mqtt_username:=user \
  -p mqtt_password:=pass
```

## Konfiguracja

### Parametry
```bash
# Ustawienie brokera
ros2 param set /mqtt_chopper_receiver_node mqtt_broker_host mss-mqtt.ddns.net

# Ustawienie topicu
ros2 param set /mqtt_chopper_receiver_node mqtt_topic_chopper test/polaczenia

# Ustawienie uwierzytelniania
ros2 param set /mqtt_chopper_receiver_node mqtt_username user
ros2 param set /mqtt_chopper_receiver_node mqtt_password pass
```

### Testowanie połączenia
```bash
# Test subskrypcji
mosquitto_sub -h mss-mqtt.ddns.net -t "test/polaczenia" -v

# Test publikacji (format skrótów)
mosquitto_pub -h mss-mqtt.ddns.net -t "test/polaczenia" -m '{"lat":52.123,"lon":21.123,"speed":2.5,"heading":45,"rtk_status":4}'
```

## Diagnostyka

### Sprawdzanie statusu
```bash
# Sprawdź węzły
ros2 node list | grep mqtt

# Sprawdź topiki
ros2 topic list | grep chopper

# Sprawdź dane GPS
ros2 topic echo /gps_rtk_data/chopper

# Sprawdź health status
ros2 topic echo /mss/node_health/mqtt_chopper_receiver
```

### Monitoring
```bash
# Sprawdź parametry
ros2 param list /mqtt_chopper_receiver_node

# Sprawdź logi
ros2 log level /mqtt_chopper_receiver_node DEBUG
```

## Zastąpienie Bluetooth

### Porównanie

| Aspekt | Bluetooth | MQTT |
|--------|-----------|------|
| **Zasięg** | ~10m | Globalny (internet) |
| **Stabilność** | Niska | Wysoka |
| **Przepustowość** | Ograniczona | Wysoka |
| **Zależności** | Hardware BT | Internet |
| **Konfiguracja** | Prosta | Wymaga brokera |

### Migracja

1. **Zatrzymaj węzeł Bluetooth:**
```bash
ros2 node kill /bt_receiver_node
```

2. **Uruchom węzeł MQTT:**
```bash
ros2 launch mqtt_comm mqtt_chopper_receiver.launch.py
```

3. **Sprawdź kompatybilność:**
```bash
ros2 topic echo /gps_rtk_data/chopper
```

## Bezpieczeństwo

### Uwierzytelnianie
- **Username/Password** - podstawowe uwierzytelnianie
- **TLS/SSL** - szyfrowanie połączenia (opcjonalne)
- **ACL** - kontrola dostępu na brokerze

### Konfiguracja bezpieczna
```bash
ros2 param set /mqtt_chopper_receiver_node mqtt_username secure_user
ros2 param set /mqtt_chopper_receiver_node mqtt_password secure_password
ros2 param set /mqtt_chopper_receiver_node mqtt_broker_port 8883  # TLS
```

## Wydajność

### Metryki
- **Częstotliwość danych**: zależna od sieczkarni
- **Opóźnienie**: ~100-500ms (internet)
- **Zużycie CPU**: ~1-3%
- **Zużycie RAM**: ~20MB

### Optymalizacja
- Dostosuj `mqtt_keepalive` do potrzeb
- Użyj QoS 0 dla szybszej transmisji
- Monitoruj `reconnect_interval`

## Troubleshooting

### Problem: Brak połączenia
```bash
# Sprawdź dostępność brokera
ping mss-mqtt.ddns.net

# Sprawdź port
telnet mss-mqtt.ddns.net 1883

# Sprawdź logi
ros2 log level /mqtt_chopper_receiver_node DEBUG
```

### Problem: Brak danych
```bash
# Sprawdź topic
ros2 topic echo /gps_rtk_data/chopper

# Sprawdź health
ros2 topic echo /mss/node_health/mqtt_chopper_receiver

# Test ręczny
mosquitto_pub -h mss-mqtt.ddns.net -t "test/polaczenia" -m '{"lat":52.123,"lon":21.123,"speed":2.5,"heading":45,"rtk_status":4}'
```

### Problem: Błędy parsowania
- Sprawdź format JSON w wiadomościach
- Waliduj wymagane pola
- Sprawdź kodowanie UTF-8

## Autorzy
- **Główny deweloper**: Adam Wróblewski
- **Email**: adam01wroblewski@gmail.com
