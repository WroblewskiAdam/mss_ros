// Plik: operator_interface/web/main.js

document.addEventListener('DOMContentLoaded', () => {
    const ROS_BRIDGE_URL = 'ws://192.168.1.40:9090'; // Upewnij się, że ten adres IP jest poprawny
    const PLACEHOLDER_FLOAT = 99999.0;

    // --- Elementy DOM ---
    const connectionStatusDiv = document.getElementById('connection-status');
    const autopilotStatusDiv = document.getElementById('autopilot-status');
    const toggleAutopilotBtn = document.getElementById('toggle-autopilot-btn');
    const vizContainer = document.getElementById('viz-container');
    const tractorElement = document.getElementById('tractor');

    // --- Połączenie z ROS ---
    const ros = new ROSLIB.Ros({ url: ROS_BRIDGE_URL });

    ros.on('connection', () => {
        connectionStatusDiv.textContent = 'POŁĄCZONO';
        connectionStatusDiv.className = 'status-bar status-on';
        toggleAutopilotBtn.disabled = false;
    });

    ros.on('error', (error) => {
        connectionStatusDiv.textContent = 'BŁĄD POŁĄCZENIA';
        connectionStatusDiv.className = 'status-bar status-off';
        toggleAutopilotBtn.disabled = true;
    });

    ros.on('close', () => {
        connectionStatusDiv.textContent = 'ROZŁĄCZONO';
        connectionStatusDiv.className = 'status-bar status-off';
        toggleAutopilotBtn.disabled = true;
    });

    // --- Subskrypcja tematu diagnostycznego ---
    const diagnosticsListener = new ROSLIB.Topic({
        ros: ros,
        name: '/diagnostics',
        messageType: 'my_robot_interfaces/msg/DiagnosticData'
    });

    const rtkStatusMap = { 0: 'BRAK', 1: 'SPS', 2: 'DGPS', 4: 'FIX', 5: 'FLOAT', 255: 'TIMEOUT' };
    const clutchStatusMap = { 0: 'Zwolnione', 1: 'WCIŚNIĘTE', 255: 'TIMEOUT' };

    diagnosticsListener.subscribe((message) => {
        // --- Aktualizacja danych ---
        updateText('bt_status', message.bt_status ? 'OK' : 'BŁĄD', message.bt_status);
        updateText('tractor_rtk', rtkStatusMap[message.tractor_gps_filtered.rtk_status] || 'Nieznany');
        updateText('chopper_rtk', rtkStatusMap[message.chopper_gps.rtk_status] || 'Nieznany');
        
        // POPRAWIONA LOGIKA: Mnożenie odbywa się wewnątrz funkcji pomocniczej, PO sprawdzeniu placeholdera
        updateFloat('dist_long', message.relative_position.distance_longitudinal, 2);
        updateFloat('dist_lat', message.relative_position.distance_lateral, 1); // Przekazujemy mnożnik jako argument
        
        updateFloat('tractor_speed', message.tractor_gps_filtered.speed_mps, 1, 3.6);
        updateFloat('target_speed', message.target_speed.data, 1, 3.6);
        
        updateText('gear', message.tractor_gear.gear === 255 ? 'TIMEOUT' : message.tractor_gear.gear);
        updateText('clutch', clutchStatusMap[message.tractor_gear.clutch_state] || 'Nieznany');

        updateFloat('tractor_lat', message.tractor_gps_filtered.latitude_deg, 6);
        updateFloat('tractor_lon', message.tractor_gps_filtered.longitude_deg, 6);
        updateFloat('chopper_lat', message.chopper_gps.latitude_deg, 6);
        updateFloat('chopper_lon', message.chopper_gps.longitude_deg, 6);

        // --- Aktualizacja wizualizacji 2D ---
        if (message.relative_position.distance_longitudinal !== PLACEHOLDER_FLOAT) {
            const scale = 20; // px na metr
            let top = (vizContainer.clientHeight / 2) - (message.relative_position.distance_longitudinal * scale);
            let left = (vizContainer.clientWidth / 2) + (message.relative_position.distance_lateral * scale);
            const relativeHeading = message.tractor_gps_filtered.heading_deg - message.chopper_gps.heading_deg;

            tractorElement.style.top = `${top}px`;
            tractorElement.style.left = `${left}px`;
            tractorElement.style.transform = `translate(-50%, -50%) rotate(${relativeHeading}deg)`;
        }
    });

    // --- Funkcje pomocnicze do aktualizacji UI ---
    function updateText(id, value, isOk) {
        const el = document.getElementById(id);
        if (el) {
            el.textContent = value;
            if (isOk !== undefined) {
                el.className = isOk ? 'value-ok' : 'value-bad';
            }
        }
    }
    
    // ZMODYFIKOWANA FUNKCJA: Przyjmuje opcjonalny mnożnik
    function updateFloat(id, value, precision, multiplier = 1.0) {
        const el = document.getElementById(id);
        if (el) {
            if (value === PLACEHOLDER_FLOAT) {
                el.textContent = 'TIMEOUT';
                el.className = 'value-bad';
            } else {
                // Mnożenie jest stosowane dopiero, gdy wiemy, że wartość jest poprawna
                el.textContent = (value * multiplier).toFixed(precision);
                el.className = 'value-ok';
            }
        }
    }

    // --- Kontrola autopilota (bez zmian) ---
    const setAutopilotClient = new ROSLIB.Service({
        ros: ros,
        name: '/speed_controller/set_enabled',
        serviceType: 'std_srvs/srv/SetBool'
    });

    let isAutopilotOn = false;
    toggleAutopilotBtn.onclick = () => {
        const targetState = !isAutopilotOn;
        const request = new ROSLIB.ServiceRequest({ data: targetState });
        setAutopilotClient.callService(request, (result) => {
            if (result.success) {
                isAutopilotOn = targetState;
                updateAutopilotUI();
            } else {
                alert("Nie udało się zmienić stanu autopilota!");
            }
        });
    };

    function updateAutopilotUI() {
        if (isAutopilotOn) {
            autopilotStatusDiv.className = 'status-bar status-on';
            autopilotStatusDiv.textContent = 'AUTOPILOT AKTYWNY';
            toggleAutopilotBtn.className = 'btn-disengage';
            toggleAutopilotBtn.textContent = 'DEZAKTYWUJ';
        } else {
            autopilotStatusDiv.className = 'status-bar status-off';
            autopilotStatusDiv.textContent = 'AUTOPILOT WYŁĄCZONY';
            toggleAutopilotBtn.className = 'btn-engage';
            toggleAutopilotBtn.textContent = 'AKTYWUJ';
        }
    }
});
