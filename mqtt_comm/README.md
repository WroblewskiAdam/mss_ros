# MQTT Communication Package

Pakiet do komunikacji MQTT z sieczkarnią, zastępujący niestabilną komunikację Bluetooth.

## Węzły

### 1. `mqtt_chopper_receiver_node` (główny)
- Odbiera dane GPS z sieczkarni przez MQTT
- Publikuje na topiku `/gps_rtk_data/chopper`
- Automatyczne ponowne połączenie
- Health monitoring

### 2. `test_mqtt_connection` (testowy)
- Skrypt testowy do sprawdzania połączenia MQTT
- Publikuje testowe dane GPS
- Pomocny w diagnostyce

## Uruchomienie

### MQTT Chopper Receiver
```bash
# Bezpośrednio
ros2 run mqtt_comm mqtt_chopper_receiver_node

# Przez launch file
ros2 launch mqtt_comm mqtt_chopper_receiver.launch.py
```

### Test połączenia
```bash
ros2 run mqtt_comm test_mqtt_connection
```

## Konfiguracja

### Parametry
- `mqtt_broker_host` - adres brokera (domyślnie: mss-mqtt.ddns.net)
- `mqtt_broker_port` - port brokera (domyślnie: 1883)
- `mqtt_topic_chopper` - topic dla danych GPS (domyślnie: test/polaczenia)
- `mqtt_username` - nazwa użytkownika (opcjonalne)
- `mqtt_password` - hasło (opcjonalne)

### Przykład
```bash
ros2 run mqtt_comm mqtt_chopper_receiver_node --ros-args \
  -p mqtt_broker_host:=mss-mqtt.ddns.net \
  -p mqtt_topic_chopper:=test/polaczenia
```

## Format danych

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

## Testowanie

### Test subskrypcji
```bash
mosquitto_sub -h mss-mqtt.ddns.net -t "test/polaczenia" -v
```

### Test publikacji
```bash
mosquitto_pub -h mss-mqtt.ddns.net -t "test/polaczenia" -m '{"lat":52.123,"lon":21.123,"speed":2.5,"heading":45,"rtk_status":4}'
```

## Zastąpienie Bluetooth

1. Zatrzymaj węzeł Bluetooth:
```bash
ros2 node kill /bt_receiver_node
```

2. Uruchom węzeł MQTT:
```bash
ros2 launch mqtt_comm mqtt_chopper_receiver.launch.py
```

3. Sprawdź dane:
```bash
ros2 topic echo /gps_rtk_data/chopper
```

## Dokumentacja

Szczegółowa dokumentacja: [mqtt_comm_docs.md](mqtt_comm_docs.md)

## Autorzy
- Adam Wróblewski (adam01wroblewski@gmail.com)
