#!/bin/bash

# Skrypt do uruchamiania interfejsu webowego MSS
# Autor: Adam Wróblewski
# Data: $(date)

echo "🚜 Uruchamianie interfejsu webowego MSS..."
echo "=========================================="

# Sprawdź czy jesteśmy w odpowiednim katalogu
if [ ! -f "index.html" ]; then
    echo "❌ Błąd: Uruchom skrypt z katalogu web/"
    echo "   cd ~/mss_ros/install/operator_interface/share/operator_interface/web/"
    exit 1
fi

# Sprawdź czy rosbridge_server jest uruchomiony
echo "🔍 Sprawdzanie statusu rosbridge_server..."
if ! pgrep -f "rosbridge_server" > /dev/null; then
    echo "⚠️  rosbridge_server nie jest uruchomiony!"
    echo "   Uruchom: ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
    echo ""
    echo "Czy chcesz uruchomić rosbridge_server teraz? (t/n)"
    read -r response
    if [[ "$response" =~ ^[Tt]$ ]]; then
        echo "🚀 Uruchamianie rosbridge_server..."
        ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
        ROSBRIDGE_PID=$!
        echo "   rosbridge_server uruchomiony (PID: $ROSBRIDGE_PID)"
        sleep 3
    else
        echo "❌ Interfejs nie może działać bez rosbridge_server"
        exit 1
    fi
else
    echo "✅ rosbridge_server jest uruchomiony"
fi

# Sprawdź dostępność portu 8000
echo "🔍 Sprawdzanie dostępności portu 8000..."
if lsof -Pi :8000 -sTCP:LISTEN -t >/dev/null ; then
    echo "⚠️  Port 8000 jest już zajęty!"
    echo "   Zatrzymuję istniejący serwer..."
    pkill -f "python3 -m http.server 8000"
    sleep 2
fi

# Uruchom serwer HTTP
echo "🌐 Uruchamianie serwera HTTP na porcie 8000..."
python3 -m http.server 8000 &
HTTP_PID=$!

echo ""
echo "✅ Interfejs webowy uruchomiony!"
echo "=========================================="
echo "📱 Otwórz przeglądarkę i przejdź do:"
echo "   http://$(hostname -I | awk '{print $1}'):8000"
echo "   lub"
echo "   http://localhost:8000"
echo ""
echo "🔧 Aby zatrzymać interfejs:"
echo "   pkill -f 'python3 -m http.server 8000'"
echo ""
echo "📊 Aby zatrzymać rosbridge_server:"
echo "   pkill -f 'rosbridge_server'"
echo ""
echo "🔄 Aby uruchomić ponownie:"
echo "   ./start_interface.sh"
echo ""

# Funkcja czyszczenia przy wyjściu
cleanup() {
    echo ""
    echo "🧹 Czyszczenie..."
    if [ ! -z "$HTTP_PID" ]; then
        kill $HTTP_PID 2>/dev/null
        echo "   Serwer HTTP zatrzymany"
    fi
    if [ ! -z "$ROSBRIDGE_PID" ]; then
        kill $ROSBRIDGE_PID 2>/dev/null
        echo "   rosbridge_server zatrzymany"
    fi
    echo "✅ Wszystko zatrzymane"
    exit 0
}

# Przechwyć sygnały wyjścia
trap cleanup SIGINT SIGTERM

echo "⏳ Naciśnij Ctrl+C aby zatrzymać..."
wait
