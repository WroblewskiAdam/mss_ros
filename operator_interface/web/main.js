// Plik: operator_interface/web/main.js

document.addEventListener('DOMContentLoaded', () => {
    const ROS_BRIDGE_URL = 'ws://192.168.1.40:9090';
    const PLACEHOLDER_FLOAT = 99999.0;
    const PLACEHOLDER_INT = 99999;
    const CHART_MAX_DATA_POINTS = 100;

    // --- Inicjalizacja ROS ---
    const ros = new ROSLIB.Ros({ url: ROS_BRIDGE_URL });

    // --- Logika zakładek ---
    window.openTab = (evt, tabName) => {
        document.querySelectorAll('.tab-content').forEach(tab => tab.style.display = 'none');
        document.querySelectorAll('.tab-button').forEach(btn => btn.className = btn.className.replace(' active', ''));
        const activeTab = document.getElementById(tabName);
        activeTab.style.display = 'flex';
        // Specjalny styl dla zakładki szczegółów
        if (tabName === 'TabDetails') {
            activeTab.style.display = 'flex'; 
        }
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
                { label: 'Prędkość zadana [m/s]', borderColor: 'red', data: [], fill: false, pointRadius: 0 },
                { label: 'Prędkość aktualna [m/s]', borderColor: 'blue', data: [], fill: false, pointRadius: 0 },
                { label: 'Sterowanie [kąt °]', borderColor: 'green', data: [], yAxisID: 'y-axis-2', fill: false, pointRadius: 0 }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            scales: {
                x: { type: 'time', time: { unit: 'second' }, ticks: { color: 'white' } },
                y: { beginAtZero: true, ticks: { color: 'white' }, title: { display: true, text: 'Prędkość [m/s]', color: 'white' } },
                'y-axis-2': { type: 'linear', position: 'right', beginAtZero: true, max: 150, ticks: { color: 'white' }, title: { display: true, text: 'Kąt [°]', color: 'white' } }
            },
            plugins: { legend: { labels: { color: 'white' } } },
            animation: false
        }
    });

    // --- Subskrypcja danych o wysokiej częstotliwości (dla wykresu) ---
    const stateListener = new ROSLIB.Topic({
        ros: ros,
        name: '/speed_controller/state',
        messageType: 'my_robot_interfaces/msg/ControllerState'
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
        controllerChart.update();
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
        updateFloat('tractor_speed_main', message.tractor_gps_filtered.speed_mps, 1, 3.6);
        updateFloat('target_speed_main', message.target_speed.data, 1, 3.6);

        // --- Aktualizacja danych w zakładce "Szczegóły" ---
        updateText('bt_status', message.bt_status ? 'OK' : 'BŁĄD', message.bt_status);
        updateText('tractor_rtk', rtkStatusMap[message.tractor_gps_filtered.rtk_status] || 'Nieznany');
        updateText('chopper_rtk', rtkStatusMap[message.chopper_gps.rtk_status] || 'Nieznany');
        updateFloat('tractor_speed', message.tractor_gps_filtered.speed_mps, 1, 3.6);
        updateFloat('target_speed', message.target_speed.data, 1, 3.6);
        updateText('gear', message.tractor_gear.gear === 255 ? 'TIMEOUT' : message.tractor_gear.gear);
        updateText('clutch', clutchStatusMap[message.tractor_gear.clutch_state] || 'Nieznany');
        updateInt('servo_pos', message.servo_position.data);
        updateFloat('dist_long', message.relative_position.distance_longitudinal, 2);
        updateFloat('dist_lat', message.relative_position.distance_lateral, 2);
        updateFloat('dist_straight', message.relative_position.distance_straight, 2);
        updateFloat('tractor_lat', message.tractor_gps_filtered.latitude_deg, 6);
        updateFloat('tractor_lon', message.tractor_gps_filtered.longitude_deg, 6);
        updateFloat('chopper_lat', message.chopper_gps.latitude_deg, 6);
        updateFloat('chopper_lon', message.chopper_gps.longitude_deg, 6);

        // --- Aktualizacja wizualizacji 2D ---
        if (message.relative_position.distance_longitudinal !== PLACEHOLDER_FLOAT) {
            const vizContainer = document.getElementById('viz-container');
            const tractorElement = document.getElementById('tractor');
            const scale = 20;
            let top = (vizContainer.clientHeight / 2) - (message.relative_position.distance_longitudinal * scale);
            let left = (vizContainer.clientWidth / 2) + (message.relative_position.distance_lateral * scale);
            tractorElement.style.top = `${top}px`;
            tractorElement.style.left = `${left}px`;
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
                alert('Parametry zaktualizowane!');
            } else {
                alert('Błąd podczas aktualizacji parametrów.');
            }
        });
    };

    // --- Pozostałe funkcje pomocnicze i logika ---
    const connectionStatusDiv = document.getElementById('connection-status');
    const autopilotStatusDiv = document.getElementById('autopilot-status');
    const toggleAutopilotBtn = document.getElementById('toggle-autopilot-btn');

    ros.on('connection', () => {
        connectionStatusDiv.textContent = 'POŁĄCZONO';
        connectionStatusDiv.className = 'status-bar status-on';
        toggleAutopilotBtn.disabled = false;
    });
    ros.on('error', () => {
        connectionStatusDiv.textContent = 'BŁĄD POŁĄCZENIA';
        connectionStatusDiv.className = 'status-bar status-off';
        toggleAutopilotBtn.disabled = true;
    });
    ros.on('close', () => {
        connectionStatusDiv.textContent = 'ROZŁĄCZONO';
        connectionStatusDiv.className = 'status-bar status-off';
        toggleAutopilotBtn.disabled = true;
    });

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
