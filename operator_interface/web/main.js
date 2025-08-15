// Plik: operator_interface/web/main.js

document.addEventListener('DOMContentLoaded', () => {
    const ROS_BRIDGE_URL = 'ws://192.168.138.7:9090';
    const PLACEHOLDER_FLOAT = 99999.0;
    const PLACEHOLDER_INT = 99999;
    const CHART_MAX_DATA_POINTS = 100;

    // --- Inicjalizacja ROS ---
    const ros = new ROSLIB.Ros({ url: ROS_BRIDGE_URL });

    // --- Logika zakładek ---
    window.openTab = (evt, tabName) => {
        document.querySelectorAll('.tab-content').forEach(tab => {
            tab.classList.remove('active');
            tab.style.display = 'none';
        });
        document.querySelectorAll('.tab-button').forEach(btn => btn.className = btn.className.replace(' active', ''));
        
        const activeTab = document.getElementById(tabName);
        activeTab.classList.add('active');
        activeTab.style.display = 'block';
        evt.currentTarget.className += ' active';
    };
    
    // Pokaż pierwszą zakładkę na starcie
    document.querySelector('.tab-button').click();

    // --- Inicjalizacja wykresu ---
    const ctx = document.getElementById('controller-chart').getContext('2d');
    const controllerChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                { label: 'Prędkość zadana [m/s]', borderColor: '#ef4444', data: [], fill: false, pointRadius: 0, borderWidth: 2 },
                { label: 'Prędkość aktualna [m/s]', borderColor: '#3b82f6', data: [], fill: false, pointRadius: 0, borderWidth: 2 },
                { label: 'Sterowanie [kąt °]', borderColor: '#10b981', data: [], yAxisID: 'y-axis-2', fill: false, pointRadius: 0, borderWidth: 2 }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            interaction: {
                intersect: false,
                mode: 'index'
            },
            scales: {
                x: { 
                    type: 'time', 
                    time: { unit: 'second' }, 
                    ticks: { color: '#cbd5e1' },
                    grid: { color: '#475569' }
                },
                y: { 
                    beginAtZero: true, 
                    ticks: { color: '#cbd5e1' }, 
                    title: { display: true, text: 'Prędkość [m/s]', color: '#cbd5e1' },
                    grid: { color: '#475569' }
                },
                'y-axis-2': { 
                    type: 'linear', 
                    position: 'right', 
                    beginAtZero: true, 
                    max: 150, 
                    ticks: { color: '#cbd5e1' }, 
                    title: { display: true, text: 'Kąt [°]', color: '#cbd5e1' },
                    grid: { display: false }
                }
            },
            plugins: { 
                legend: { 
                    labels: { color: '#cbd5e1' },
                    position: 'top'
                } 
            },
            animation: false
        }
    });

    // --- Modal z nastawami ---
    const modal = document.getElementById('settings-modal');
    const settingsBtn = document.getElementById('settings-btn');
    const closeBtn = document.querySelector('.close');

    settingsBtn.onclick = () => modal.style.display = 'block';
    closeBtn.onclick = () => modal.style.display = 'none';
    window.onclick = (event) => {
        if (event.target === modal) modal.style.display = 'none';
    };

    // --- Subskrypcja danych o wysokiej częstotliwości (dla wykresu) ---
    const stateListener = new ROSLIB.Topic({
        ros: ros,
        name: '/speed_controller/state',
        messageType: 'my_robot_interfaces/msg/SpeedControllerState'
    });

    stateListener.subscribe((message) => {
        const now = new Date(message.header.stamp.sec * 1000 + message.header.stamp.nanosec / 1000000);
        
        controllerChart.data.labels.push(now);
        controllerChart.data.datasets[0].data.push(message.setpoint_speed);
        controllerChart.data.datasets[1].data.push(message.current_speed);
        controllerChart.data.datasets[2].data.push(message.control_output);

        if (controllerChart.data.labels.length > CHART_MAX_DATA_POINTS) {
            controllerChart.data.labels.shift();
            controllerChart.data.datasets.forEach(dataset => dataset.data.shift());
        }
        controllerChart.update('none');
    });

    // --- Subskrypcja danych o niskiej częstotliwości (dla paneli) ---
    const diagnosticsListener = new ROSLIB.Topic({
        ros: ros,
        name: '/diagnostics',
        messageType: 'my_robot_interfaces/msg/DiagnosticData'
    });
    
    const rtkStatusMap = { 0: 'BRAK', 1: 'SPS', 2: 'DGPS', 4: 'FIX', 5: 'FLOAT', 255: 'TIMEOUT' };
    const clutchStatusMap = { 0: 'Zwolnione', 1: 'WCIŚNIĘTE', 255: 'TIMEOUT' };

    diagnosticsListener.subscribe((message) => {
        // --- Aktualizacja danych w panelu głównym ---
        updateFloat('dist_long_main', message.relative_position.distance_longitudinal, 2);
        updateFloat('dist_lat_main', message.relative_position.distance_lateral, 2);
        updateFloat('tractor_speed_main', message.tractor_gps_filtered.speed_mps, 4, 3.6);
        updateFloat('target_speed_main', message.target_speed.data, 4, 3.6);

        // --- Aktualizacja danych w zakładce "Szczegóły" ---
        // Status Systemu
        updateText('bt_status', message.bt_status ? 'OK' : 'BŁĄD', message.bt_status);
        updateText('tractor_rtk', rtkStatusMap[message.tractor_gps_filtered.rtk_status] || 'Nieznany');
        updateText('chopper_rtk', rtkStatusMap[message.chopper_gps.rtk_status] || 'Nieznany');
        
        // Ciągnik - wszystkie dostępne dane
        updateFloat('tractor_speed', message.tractor_gps_filtered.speed_mps, 4, 3.6);
        updateFloat('target_speed', message.target_speed.data, 4, 3.6);
        updateText('gear', message.tractor_gear.gear === 255 ? 'TIMEOUT' : message.tractor_gear.gear);
        updateText('clutch', clutchStatusMap[message.tractor_gear.clutch_state] || 'Nieznany');
        updateInt('servo_pos', message.servo_position.data);
        
        // Dane GPS ciągnika z wysoką rozdzielczością
        updateFloat('tractor_lat', message.tractor_gps_filtered.latitude_deg, 10);
        updateFloat('tractor_lon', message.tractor_gps_filtered.longitude_deg, 10);
        updateFloat('tractor_alt', message.tractor_gps_filtered.altitude_m, 2);
        updateFloat('tractor_heading', message.tractor_gps_filtered.heading_deg, 4);
        updateGPSTime('tractor_gps_time', message.tractor_gps_filtered.gps_time);
        
        // Sieczkarnia - wszystkie dostępne dane
        updateFloat('chopper_speed', message.chopper_gps.speed_mps, 4, 3.6);
        updateFloat('chopper_lat', message.chopper_gps.latitude_deg, 10);
        updateFloat('chopper_lon', message.chopper_gps.longitude_deg, 10);
        updateFloat('chopper_alt', message.chopper_gps.altitude_m, 2);
        updateFloat('chopper_heading', message.chopper_gps.heading_deg, 4);
        updateGPSTime('chopper_gps_time', message.chopper_gps.gps_time);
        
        // Pozycja względna
        updateFloat('dist_long', message.relative_position.distance_longitudinal, 2);
        updateFloat('dist_lat', message.relative_position.distance_lateral, 2);
        updateFloat('dist_straight', message.relative_position.distance_straight, 2);

        // --- Aktualizacja wizualizacji 2D ---
        if (message.relative_position.distance_longitudinal !== PLACEHOLDER_FLOAT) {
            const vizContainer = document.getElementById('viz-container');
            const tractorElement = document.getElementById('tractor');
            const chopperElement = document.getElementById('chopper');
            const scale = 20;
            
            // Pozycja ciągnika względem sieczkarni
            let top = (vizContainer.clientHeight / 2) - (message.relative_position.distance_longitudinal * scale);
            let left = (vizContainer.clientWidth / 2) + (message.relative_position.distance_lateral * scale);
            tractorElement.style.top = `${top}px`;
            tractorElement.style.left = `${left}px`;
            
            // Pozycjonowanie pojazdów - bez obracania
        }
        
        // --- Aktualizacja etykiet prędkości ---
        if (message.tractor_gps_filtered.speed_mps !== PLACEHOLDER_FLOAT) {
            const tractorSpeedEl = document.getElementById('tractor-speed');
            if (tractorSpeedEl) {
                const speedKmh = (message.tractor_gps_filtered.speed_mps * 3.6).toFixed(4);
                tractorSpeedEl.textContent = `${speedKmh} km/h`;
            }
        }
        
        if (message.chopper_gps.speed_mps !== PLACEHOLDER_FLOAT) {
            const chopperSpeedEl = document.getElementById('chopper-speed');
            if (chopperSpeedEl) {
                const speedKmh = (message.chopper_gps.speed_mps * 3.6).toFixed(4);
                chopperSpeedEl.textContent = `${speedKmh} km/h`;
            }
        }
    });

    // --- Logika strojenia PID ---
    const kpSlider = document.getElementById('kp-slider');
    const kiSlider = document.getElementById('ki-slider');
    const kpValueSpan = document.getElementById('kp-value');
    const kiValueSpan = document.getElementById('ki-value');

    kpSlider.oninput = () => kpValueSpan.textContent = parseFloat(kpSlider.value).toFixed(1);
    kiSlider.oninput = () => kiValueSpan.textContent = parseFloat(kiSlider.value).toFixed(1);
    
    const setParamsClient = new ROSLIB.Service({
        ros: ros,
        name: '/speed_controller_node/set_parameters',
        serviceType: 'rcl_interfaces/srv/SetParameters'
    });

    document.getElementById('apply-pid-btn').onclick = () => {
        const params = [
            { name: 'kp', value: { type: 2, double_value: parseFloat(kpSlider.value) } },
            { name: 'ki', value: { type: 2, double_value: parseFloat(kiSlider.value) } }
        ];
        const request = new ROSLIB.ServiceRequest({ parameters: params });
        setParamsClient.callService(request, (result) => {
            if (result.results.every(r => r.successful)) {
                showNotification('Parametry zaktualizowane!', 'success');
                modal.style.display = 'none';
            } else {
                showNotification('Błąd podczas aktualizacji parametrów.', 'error');
            }
        });
    };

    // --- Status połączenia i autopilota ---
    const connectionStatusDiv = document.getElementById('connection-status');
    const autopilotStatusDiv = document.getElementById('autopilot-status');
    const toggleAutopilotBtn = document.getElementById('toggle-autopilot-btn');

    ros.on('connection', () => {
        connectionStatusDiv.textContent = 'POŁĄCZONO';
        connectionStatusDiv.className = 'status-indicator status-on';
        toggleAutopilotBtn.disabled = false;
        showNotification('Połączono z ROS Bridge', 'success');
    });
    
    ros.on('error', () => {
        connectionStatusDiv.textContent = 'BŁĄD POŁĄCZENIA';
        connectionStatusDiv.className = 'status-indicator status-off';
        toggleAutopilotBtn.disabled = true;
        showNotification('Błąd połączenia z ROS Bridge', 'error');
    });
    
    ros.on('close', () => {
        connectionStatusDiv.textContent = 'ROZŁĄCZONO';
        connectionStatusDiv.className = 'status-indicator status-off';
        toggleAutopilotBtn.disabled = true;
        showNotification('Rozłączono z ROS Bridge', 'warning');
    });

    // --- Autopilot ---
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
                showNotification(
                    isAutopilotOn ? 'Autopilot aktywowany' : 'Autopilot dezaktywowany', 
                    isAutopilotOn ? 'success' : 'warning'
                );
            } else {
                showNotification("Nie udało się zmienić stanu autopilota!", 'error');
            }
        });
    };

    function updateAutopilotUI() {
        if (isAutopilotOn) {
            autopilotStatusDiv.className = 'status-indicator status-on';
            autopilotStatusDiv.textContent = 'AUTOPILOT AKTYWNY';
            toggleAutopilotBtn.className = 'btn-disengage';
            toggleAutopilotBtn.textContent = 'DEZAKTYWUJ';
        } else {
            autopilotStatusDiv.className = 'status-indicator status-off';
            autopilotStatusDiv.textContent = 'AUTOPILOT WYŁĄCZONY';
            toggleAutopilotBtn.className = 'btn-engage';
            toggleAutopilotBtn.textContent = 'AKTYWUJ';
        }
    }

    // --- Funkcje pomocnicze ---
    function updateText(id, value, isOk) {
        const el = document.getElementById(id);
        if (el) {
            el.textContent = value;
            if (isOk !== undefined) {
                el.className = isOk ? 'value-ok' : 'value-bad';
            }
        }
    }
    
    function updateFloat(id, value, precision, multiplier = 1.0) {
        const el = document.getElementById(id);
        if (el) {
            if (value === PLACEHOLDER_FLOAT) {
                el.textContent = 'TIMEOUT';
                el.className = 'value-bad';
            } else {
                el.textContent = (value * multiplier).toFixed(precision);
                el.className = 'value-ok';
            }
        }
    }

    function updateInt(id, value) {
        const el = document.getElementById(id);
        if (el) {
            if (value === PLACEHOLDER_INT) {
                el.textContent = 'TIMEOUT';
                el.className = 'value-bad';
            } else {
                el.textContent = value;
                el.className = 'value-ok';
            }
        }
    }

    function updateGPSTime(id, gpsTimeMsg) {
        const el = document.getElementById(id);
        if (el) {
            if (!gpsTimeMsg || gpsTimeMsg.sec === 0) {
                el.textContent = 'TIMEOUT';
                el.className = 'value-bad';
            } else {
                try {
                    // Konwertuj czas ROS na JavaScript Date
                    const timestamp = gpsTimeMsg.sec * 1000 + gpsTimeMsg.nanosec / 1000000;
                    const date = new Date(timestamp);
                    
                    // Formatuj czas w czytelny sposób
                    const timeString = date.toLocaleTimeString('pl-PL', {
                        hour: '2-digit',
                        minute: '2-digit',
                        second: '2-digit',
                        hour12: false
                    });
                    
                    el.textContent = timeString;
                    el.className = 'value-ok';
                } catch (error) {
                    el.textContent = 'BŁĄD';
                    el.className = 'value-bad';
                }
            }
        }
    }

    function showNotification(message, type = 'info') {
        // Prosta implementacja powiadomień
        console.log(`[${type.toUpperCase()}] ${message}`);
        
        // Można dodać bardziej zaawansowane powiadomienia
        if (type === 'error') {
            console.error(message);
        } else if (type === 'warning') {
            console.warn(message);
        }
    }
});
