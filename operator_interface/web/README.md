# MSS Operator Interface - Nowoczesny Dashboard

## 🎯 **Zakładka System - Dashboard Metryk**

### **Karty Metryk Systemowych**
- **CPU** 🖥️ - Użycie procesora w czasie rzeczywistym z mini wykresem
- **RAM** 💾 - Użycie pamięci z trendem
- **Temperatura** 🌡️ - Monitorowanie temperatury RPi
- **Dysk** 💿 - Użycie przestrzeni dyskowej

### **Informacje Systemowe**
- **Uptime** - Czas pracy systemu
- **Wersja ROS** - Informacje o ROS2
- **Architektura** - Szczegóły systemu
- **Model RPi** - Informacje o Raspberry Pi

### **Status Komponentów**
- **GPIO** - Status pinów GPIO
- **Sieć** - Status połączeń sieciowych
- **USB/Serial** - Dostępne porty
- **ROS Bridge** - Status połączenia z ROS

### **Wykresy Systemowe**
- **Wykres Wydajności** - CPU, RAM, temperatura w czasie
- **Kontrolki Czasu** - 1h, 6h, 24h
- **Aktualizacja w czasie rzeczywistym**

---

## 🏥 **Zakładka Health - Dashboard Monitorowania**

### **Status Systemu MSS**
- **Wskaźnik Ogólny** - Kolorowy status systemu (OK/WARNING/ERROR)
- **Statystyki** - Liczba węzłów aktywnych, błędów, ostrzeżeń
- **Aktualizacja w czasie rzeczywistym**

### **Status Węzłów ROS**
- **Karty Węzłów** - Każdy węzeł ma własną kartę z ikoną
- **Wskaźniki Health** - Kolorowe kropki statusu
- **Ikony Tematyczne** - 🛰️ GPS, 📱 Bluetooth, ⚙️ Biegi, 🎛️ Serwo
- **Status w czasie rzeczywistym**

### **Konsola Logów Systemu**
- **Filtrowanie** - Poziom logów (Info, Warning, Error)
- **Kontrolki** - Wyczyść, Eksportuj
- **Formatowanie** - Kolorowe logi według poziomu
- **Autoscroll** - Automatyczne przewijanie do najnowszych

### **Wykresy Health**
- **Status Węzłów** - Trend aktywnych/błędnych/ostrzeżeń
- **Kontrolki Czasu** - 1h, 6h, 24h
- **Aktualizacja w czasie rzeczywistym**

---

## 🚀 **Funkcje Dashboard**

### **Wykresy Mini**
- Każda karta metryki ma mini wykres trendu
- Maksymalnie 20 punktów danych
- Aktualizacja w czasie rzeczywistym
- Animowane przejścia

### **Responsywność**
- **Desktop** - Grid 4 kolumny dla metryk
- **Tablet** - Grid 2 kolumny
- **Mobile** - Grid 1 kolumna
- **Adaptacyjne** - Automatyczne dostosowanie

### **Interaktywność**
- **Hover Effects** - Karty unoszą się przy najechaniu
- **Animacje** - Płynne przejścia i transformacje
- **Kontrolki** - Przyciski i selektory
- **Filtry** - Filtrowanie logów i danych

---

## 🔧 **Technologie**

### **Frontend**
- **HTML5** - Semantyczna struktura
- **CSS3** - Grid, Flexbox, Animacje
- **JavaScript ES6+** - Moduły, async/await
- **Chart.js** - Wykresy interaktywne

### **Integracja ROS**
- **roslib.js** - ROS2 Bridge
- **WebSocket** - Komunikacja w czasie rzeczywistym
- **JSON** - Parsowanie wiadomości ROS
- **Health Topics** - Subskrypcje statusu

---

## 📱 **Użycie**

### **Monitoring Systemu**
1. Przejdź do zakładki **System**
2. Sprawdź karty metryk (CPU, RAM, temperatura, dysk)
3. Przejrzyj szczegółowe informacje systemowe
4. Analizuj wykresy wydajności

### **Monitoring Health**
1. Przejdź do zakładki **Health**
2. Sprawdź ogólny status systemu MSS
3. Przejrzyj status poszczególnych węzłów
4. Monitoruj logi w konsoli
5. Analizuj trendy w wykresach

### **Konsola Logów**
1. **Filtruj** - Wybierz poziom logów
2. **Czyść** - Usuń stare logi
3. **Eksportuj** - Pobierz logi jako plik TXT
4. **Monitoruj** - Obserwuj logi w czasie rzeczywistym

---

## 🎨 **Design System**

### **Kolory**
- **Primary** - #3498db (niebieski)
- **Success** - #27ae60 (zielony)
- **Warning** - #f39c12 (pomarańczowy)
- **Error** - #e74c3c (czerwony)
- **Info** - #95a5a6 (szary)

### **Typografia**
- **Nagłówki** - Montserrat, 1.3em
- **Tekst** - Open Sans, 1em
- **Konsola** - Courier New, monospace
- **Ikony** - Emoji i Font Awesome

### **Spacing**
- **Padding** - 20px (desktop), 15px (mobile)
- **Gap** - 20px (desktop), 15px (mobile)
- **Margin** - 30px (sekcje), 15px (elementy)

---

## 🔄 **Aktualizacje**

### **Częstotliwość**
- **Metryki Systemowe** - Co 5 sekund
- **Status Węzłów** - Co 5 sekund
- **Wykresy** - W czasie rzeczywistym
- **Logi** - Natychmiastowo

### **Źródła Danych**
- **System Monitor** - `/mss/node_health/system_monitor`
- **Health Monitor** - `/mss/system_status`
- **Node Health** - `/mss/node_health/{node_name}`
- **Health Alerts** - `/mss/health_alerts`

---

## 🐛 **Rozwiązywanie Problemów**

### **Wykresy nie działają**
- Sprawdź czy Chart.js jest załadowany
- Sprawdź konsolę przeglądarki
- Upewnij się że canvas ma odpowiednie ID

### **Dane nie aktualizują się**
- Sprawdź połączenie ROS Bridge
- Sprawdź czy topiki są aktywne
- Sprawdź logi węzłów ROS

### **Responsywność nie działa**
- Sprawdź CSS media queries
- Sprawdź czy viewport meta tag jest ustawiony
- Przetestuj na różnych urządzeniach

---

## 📈 **Rozwój**

### **Planowane Funkcje**
- **Eksport Wykresów** - PNG, PDF
- **Alerty Push** - Powiadomienia przeglądarki
- **Historia Danych** - Długoterminowe trendy
- **Konfiguracja** - Ustawienia dashboard
- **Tematy** - Jasny/ciemny motyw

### **Optymalizacje**
- **Lazy Loading** - Wykresy ładowane na żądanie
- **Web Workers** - Przetwarzanie danych w tle
- **Service Worker** - Offline support
- **PWA** - Progressive Web App
