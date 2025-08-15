# 🚜 MSS Operator Interface

Nowoczesny, mobilny interfejs webowy dla systemu synchronizacji ciągnika rolniczego ze sieczkarnią.

## ✨ Funkcje

- **📱 Responsywny design** - optymalizowany dla telefonów w trybie pionowym
- **🌐 Komunikacja ROS2** - przez rosbridge_server
- **📊 Wizualizacja w czasie rzeczywistym** - pozycja względna, prędkość, status
- **⚙️ Kontrola autopilota** - włączanie/wyłączanie systemu
- **📈 Wykresy regulatora** - monitoring pracy regulatora PI
- **🔧 Nastawy regulatora** - dynamiczna zmiana parametrów Kp/Ki

## 🚀 Szybkie uruchomienie

### Opcja 1: Automatyczne uruchomienie (zalecane)
```bash
cd ~/mss_ros/install/operator_interface/share/operator_interface/web/
./start_interface.sh
```

### Opcja 2: Ręczne uruchomienie
```bash
# Terminal 1: Uruchom rosbridge_server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Terminal 2: Uruchom serwer HTTP
cd ~/mss_ros/install/operator_interface/share/operator_interface/web/
python3 -m http.server 8000
```

## 📱 Dostęp do interfejsu

Otwórz przeglądarkę i przejdź do:
- **Lokalnie**: `http://localhost:8000`
- **Z sieci**: `http://[IP_RPI]:8000`

## 🎯 Zakładki interfejsu

### 🏠 Główna
- **Wizualizacja 2D** - pozycja ciągnika względem sieczkarni z ładnymi symbolami SVG
- **Symbole pojazdów** - ciągnik z przyczepą i sieczkarnia z detalami
- **Strzałki prędkości** - obracają się zgodnie z kierunkiem ruchu
- **Etykiety prędkości** - wyświetlają aktualną prędkość w km/h
- **Kluczowe dane** - odległości, prędkości w czasie rzeczywistym
- **Status połączenia** - stan komunikacji z ROS2

### 📊 Regulator
- **Wykres pracy** - prędkość zadana, aktualna i sterowanie
- **🎯 Ustawianie prędkości zadanej** - pole input + przycisk ustawienia
- **Wyświetlacz aktualnej prędkości** - monitoring w czasie rzeczywistym
- **Nastawy** - przycisk otwierający modal z parametrami Kp/Ki
- **Pełnoekranowy wykres** - optymalny do analizy

### 🎮 Control
- **🚜 Kontrola Regulatora Prędkości** - status, włączanie/wyłączanie regulacji
- **⚙️ Kontrola Biegów** - status gear manager, automatyczne/rzęczne zarządzanie, zmiana biegów
- **🎮 Ręczne Sterowanie Serwem** - ustawianie pozycji, szybkie pozycje (lewo/środek/prawo)
- **📊 Status Systemu** - połączenie ROS, ostatnie komendy, status serwa
- **Ręczne sterowanie** - bezpośrednie zarządzanie systemem

### 🔍 Szczegóły
- **🔧 Status Systemu** - połączenie BT, status RTK
- **🚜 Ciągnik** - prędkość, biegi, sprzęgło, serwo, pozycja GPS, wysokość, kurs, czas GPS
- **🌾 Sieczkarnia** - prędkość, pozycja GPS, wysokość, kurs, czas GPS
- **📍 Pozycja Względna** - odległości wzdłużne i poprzeczne

## ⚙️ Konfiguracja

### Adres ROS Bridge
W pliku `main.js` linia 4:
```javascript
const ROS_BRIDGE_URL = 'ws://192.168.138.7:9090';
```

### Rozdzielczość danych
- **GPS (lat/lon)**: 10 miejsc po przecinku
- **Prędkość**: 4 miejsca po przecinku
- **Kurs**: 4 miejsca po przecinku
- **Odległości**: 2 miejsca po przecinku
- **Wysokość**: 2 miejsca po przecinku

## 🛠️ Rozwój

### Struktura plików
```
web/
├── index.html          # Struktura HTML
├── style.css           # Style CSS (nowoczesne, mobilne)
├── main.js             # Logika JavaScript
├── start_interface.sh  # Skrypt uruchamiania
└── README.md           # Ta dokumentacja
```

### Technologie
- **HTML5** - semantyczna struktura
- **CSS3** - Grid, Flexbox, CSS Variables, Media Queries
- **JavaScript ES6+** - nowoczesna składnia
- **Chart.js** - wykresy w czasie rzeczywistym
- **ROSLIB.js** - komunikacja z ROS2

## 📱 Responsywność

Interfejs automatycznie dostosowuje się do:
- **Desktop** (>768px) - układ poziomy
- **Tablet** (768px) - układ mieszany
- **Telefon** (<480px) - układ pionowy, większe elementy

## 🔧 Rozwiązywanie problemów

### Interfejs nie łączy się z ROS2
1. Sprawdź czy `rosbridge_server` jest uruchomiony
2. Sprawdź adres IP w `main.js`
3. Sprawdź firewall i dostępność portu 9090

### Dane nie są aktualizowane
1. Sprawdź czy topik `/diagnostics` jest publikowany
2. Sprawdź logi w konsoli przeglądarki (F12)
3. Sprawdź połączenie WebSocket

### Problemy z responsywnością
1. Odśwież stronę (Ctrl+F5)
2. Sprawdź czy viewport meta tag jest poprawny
3. Sprawdź CSS media queries

## 📞 Wsparcie

W przypadku problemów:
1. Sprawdź logi w konsoli przeglądarki
2. Sprawdź logi ROS2: `ros2 topic echo /diagnostics`
3. Sprawdź status usług: `systemctl status rosbridge_server`

## 🔄 Aktualizacje

Aby zaktualizować interfejs:
```bash
cd ~/mss_ros/src
colcon build --packages-select operator_interface
source install/setup.bash
```

---

**Autor**: Adam Wróblewski  
**Wersja**: 2.0  
**Data**: $(date)
