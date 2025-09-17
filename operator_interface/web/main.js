// Plik: operator_interface/web/main.js

document.addEventListener('DOMContentLoaded', () => {
    const ROS_BRIDGE_URL = 'ws://192.168.1.77:9090';
    const PLACEHOLDER_FLOAT = 99999.0;
    const PLACEHOLDER_INT = 99999;
    const CHART_MAX_DATA_POINTS = 100;

    // --- Zmienne stanu systemu ---
    let isAutopilotOn = false;           // Główny autopilot (regulator prędkości + pozycji)
    let isSpeedControllerEnabled = false; // Regulator prędkości
    let isPositionControllerEnabled = false; // Regulator pozycji (na razie nieistniejący)
    
    // --- Zmienne skalowania ikon - ZMIEŃ TUTAJ ROZMIARY ---
    const CHOPPER_SCALE = 1.4;  // Skala sieczkarni (0.1 = 10%, 1.0 = 100%, 2.0 = 200%)
    const TRACTOR_SCALE = 2.3;  // Skala ciągnika (0.1 = 10%, 1.0 = 100%, 2.0 = 200%)
    const BASE_CHOPPER_SIZE = { width: 100, height: 100 }; // Bazowy rozmiar sieczkarni
    const BASE_TRACTOR_SIZE = { width: 80, height: 200 };  // Bazowy rozmiar ciągnika
    
    // --- OFFSETY PUNKTÓW REFERENCYJNYCH - ZMIEŃ TUTAJ POZYCJE ---
    const CHOPPER_OFFSET = { x: 3, y: 8 };    // Offset dla punktu 0,0 sieczkarni (rura)
    const TRACTOR_OFFSET = { x: 3, y: -25 };    // Offset dla punktu 0,0 ciągnika (środek przyczepy)
    
    // --- PUNKTY REFERENCYJNE - WŁĄCZ/WYŁĄCZ ---
    const SHOW_REFERENCE_POINTS = true;  // true = pokaż punkty, false = ukryj

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

    // --- Inicjalizacja rozmiarów ikon i punktów referencyjnych ---
    function initializeIconSizes() {
        // Ustaw rozmiary sieczkarni
        const chopperIcon = document.querySelector('#chopper .vehicle-icon');
        if (chopperIcon) {
            chopperIcon.style.width = `${BASE_CHOPPER_SIZE.width * CHOPPER_SCALE}px`;
            chopperIcon.style.height = `${BASE_CHOPPER_SIZE.height * CHOPPER_SCALE}px`;
        }
        
        // Ustaw rozmiary ciągnika
        const tractorIcon = document.querySelector('#tractor .vehicle-icon');
        if (tractorIcon) {
            tractorIcon.style.width = `${BASE_TRACTOR_SIZE.width * TRACTOR_SCALE}px`;
            tractorIcon.style.height = `${BASE_TRACTOR_SIZE.height * TRACTOR_SCALE}px`;
        }
        
        // Inicjalizuj punkty referencyjne
        initializeReferencePoints();
        
        console.log(`Rozmiary ikon zainicjalizowane - Sieczkarnia: ${CHOPPER_SCALE * 100}%, Ciągnik: ${TRACTOR_SCALE * 100}%`);
    }
    
    // --- Inicjalizacja punktów referencyjnych ---
    function initializeReferencePoints() {
        const chopperRefPoint = document.getElementById('chopper-ref-point');
        const tractorRefPoint = document.getElementById('tractor-ref-point');
        
        if (chopperRefPoint) {
            chopperRefPoint.style.display = SHOW_REFERENCE_POINTS ? 'block' : 'none';
        }
        
        if (tractorRefPoint) {
            tractorRefPoint.style.display = SHOW_REFERENCE_POINTS ? 'block' : 'none';
        }
        
        console.log(`Punkty referencyjne ${SHOW_REFERENCE_POINTS ? 'włączone' : 'wyłączone'}`);
    }
    
    // Inicjalizacja rozmiarów na starcie
    initializeIconSizes();

    // --- Inicjalizacja wykresu ---
    const ctx = document.getElementById('controller-chart').getContext('2d');
    const controllerChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                { label: 'Prędkość zadana [km/h]', borderColor: '#ef4444', data: [], fill: false, pointRadius: 0, borderWidth: 2 },
                { label: 'Prędkość aktualna [km/h]', borderColor: '#3b82f6', data: [], fill: false, pointRadius: 0, borderWidth: 2 },
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
                    title: { display: true, text: 'Prędkość [km/h]', color: '#cbd5e1' },
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

    // --- Inicjalizacja wykresu regulatora pozycji ---
    const positionCtx = document.getElementById('position-controller-chart').getContext('2d');
    const positionControllerChart = new Chart(positionCtx, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                { label: 'Pozycja zadana [m]', borderColor: '#ef4444', data: [], fill: false, pointRadius: 0, borderWidth: 2 },
                { label: 'Pozycja aktualna [m]', borderColor: '#3b82f6', data: [], fill: false, pointRadius: 0, borderWidth: 2 },
                { label: 'Sterowanie [m/s]', borderColor: '#10b981', data: [], yAxisID: 'y-axis-2', fill: false, pointRadius: 0, borderWidth: 2 }
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
                    beginAtZero: false, 
                    ticks: { color: '#cbd5e1' }, 
                    title: { display: true, text: 'Pozycja [m]', color: '#cbd5e1' },
                    grid: { color: '#475569' }
                },
                'y-axis-2': { 
                    type: 'linear', 
                    position: 'right', 
                    beginAtZero: true, 
                    max: 10, 
                    ticks: { color: '#cbd5e1' }, 
                    title: { display: true, text: 'Prędkość [m/s]', color: '#cbd5e1' },
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
    
    // --- Modal z nastawami regulatora pozycji ---
    const positionModal = document.getElementById('position-settings-modal');
    const positionSettingsBtn = document.getElementById('position-settings-btn');
    const positionCloseBtn = positionModal ? positionModal.querySelector('.close') : null;
    


    settingsBtn.onclick = () => modal.style.display = 'block';
    closeBtn.onclick = () => modal.style.display = 'none';
    
    if (positionSettingsBtn) {
        positionSettingsBtn.onclick = () => positionModal.style.display = 'block';
    }
    if (positionCloseBtn) {
        positionCloseBtn.onclick = () => positionModal.style.display = 'none';
    }
    

    window.onclick = (event) => {
        if (event.target === modal) modal.style.display = 'none';
        if (event.target === positionModal) positionModal.style.display = 'none';
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
        controllerChart.data.datasets[0].data.push(message.setpoint_speed * 3.6); // m/s -> km/h
        controllerChart.data.datasets[1].data.push(message.current_speed * 3.6); // m/s -> km/h
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

    // --- Subskrypcje dla regulatora pozycji ---
    const autopilotStatusListener = new ROSLIB.Topic({
        ros: ros,
        name: '/autopilot/status',
        messageType: 'std_msgs/msg/String'
    });

    const distanceMetricsListener = new ROSLIB.Topic({
        ros: ros,
        name: '/distance_metrics',
        messageType: 'my_robot_interfaces/msg/DistanceMetrics'
    });

    // --- Callback functions dla regulatora pozycji ---
    autopilotStatusListener.subscribe((message) => {
        const statusElement = document.getElementById('autopilot_status');
        if (statusElement) {
            statusElement.textContent = message.data;
        }
    });

    distanceMetricsListener.subscribe((message) => {
        const distanceElement = document.getElementById('current_distance_longitudinal');
        if (distanceElement) {
            distanceElement.textContent = message.distance_longitudinal.toFixed(2) + ' m';
        }
        
        // Aktualizacja wykresu regulatora pozycji
        if (window.lastTargetSpeed !== undefined) {
            const now = new Date();
            positionControllerChart.data.labels.push(now);
            positionControllerChart.data.datasets[0].data.push(window.lastTargetPosition || 0.0);
            positionControllerChart.data.datasets[1].data.push(message.distance_longitudinal);
            positionControllerChart.data.datasets[2].data.push(window.lastTargetSpeed);

            if (positionControllerChart.data.labels.length > CHART_MAX_DATA_POINTS) {
                positionControllerChart.data.labels.shift();
                positionControllerChart.data.datasets.forEach(dataset => dataset.data.shift());
            }
            positionControllerChart.update('none');
        }
    });

    // --- Subskrypcja prędkości zadanej przez regulator pozycji ---
    const targetSpeedListener = new ROSLIB.Topic({
        ros: ros,
        name: '/target_speed',
        messageType: 'std_msgs/msg/Float64'
    });

    targetSpeedListener.subscribe((message) => {
        window.lastTargetSpeed = message.data;
    });

    const rtkStatusMap = { 0: 'BRAK', 1: 'SPS', 2: 'DGPS', 4: 'FIX', 5: 'FLOAT', 255: 'TIMEOUT' };
    const clutchStatusMap = { 0: 'Zwolnione', 1: 'WCIŚNIĘTE', 255: 'TIMEOUT' };

    diagnosticsListener.subscribe((message) => {
        // --- Aktualizacja danych w panelu głównym ---
        updateFloat('dist_long_main', message.relative_position.distance_longitudinal, 2);
        updateFloat('dist_lat_main', message.relative_position.distance_lateral, 2);
        updateFloat('dist_straight_main', message.relative_position.distance_straight, 2);
        updateFloat('tractor_speed_main', message.tractor_gps_filtered.speed_mps, 4, 3.6);
        updateFloat('target_speed_main', message.target_speed.data, 4, 3.6);
        
        // Aktualizacja biegu w panelu głównym
        const gearValue = message.tractor_gear.gear === 255 ? '---' : message.tractor_gear.gear;
        updateText('current_gear_main', gearValue);

        // --- Aktualizacja danych w zakładce "Szczegóły" ---
        // Status Systemu
        updateText('bt_status', message.bt_status ? 'OK' : 'BŁĄD', message.bt_status);
        updateText('tractor_rtk', rtkStatusMap[message.tractor_gps_filtered.rtk_status] || 'Nieznany');
        updateText('chopper_rtk', rtkStatusMap[message.chopper_gps.rtk_status] || 'Nieznany');
        
        // Ciągnik - wszystkie dostępne dane
        updateFloat('tractor_speed_details', message.tractor_gps_filtered.speed_mps, 4, 3.6);
        updateFloat('target_speed', message.target_speed.data, 4, 3.6);
        
        // Prędkość aktualna jest teraz wyświetlana tylko na wykresie
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

        // Zapisz ostatnią wiadomość dla ponownego wywołania wizualizacji
        window.lastDiagnosticMessage = message;

        // --- Aktualizacja wizualizacji 2D ---
        updateVisualization(message);
        
        // --- Aktualizacja etykiet prędkości ---
        if (message.tractor_gps_filtered.speed_mps !== PLACEHOLDER_FLOAT) {
            const tractorSpeedEl = document.getElementById('tractor-speed');
            if (tractorSpeedEl) {
                const speedKmh = (message.tractor_gps_filtered.speed_mps * 3.6).toFixed(2);
                tractorSpeedEl.textContent = `${speedKmh} km/h`;
            }
        }
        
        if (message.chopper_gps.speed_mps !== PLACEHOLDER_FLOAT) {
            const chopperSpeedEl = document.getElementById('chopper-speed');
            if (chopperSpeedEl) {
                const speedKmh = (message.chopper_gps.speed_mps * 3.6).toFixed(2);
                chopperSpeedEl.textContent = `${speedKmh} km/h`;
            }
        }
    });

    // Funkcja aktualizacji wizualizacji 2D
    function updateVisualization(message) {
        if (message.relative_position.distance_longitudinal !== PLACEHOLDER_FLOAT) {
            const vizContainer = document.getElementById('viz-container');
            const tractorElement = document.getElementById('tractor');
            const chopperElement = document.getElementById('chopper');
            
            // 3. Realistyczne skalowanie - jak prawdziwe podjeżdżanie
            const maxDistance = Math.max(
                Math.abs(message.relative_position.distance_longitudinal),
                Math.abs(message.relative_position.distance_lateral)
            );
            
            // 1. Ustawienie marginesów i dostępnej przestrzeni
            const margin = 25; // Margines z każdej strony
            const containerCenterX = vizContainer.clientWidth / 2;
            const containerCenterY = vizContainer.clientHeight / 2;
            
            // Dostępna przestrzeń z marginesami
            const availableWidth = vizContainer.clientWidth - (margin * 2);
            const availableHeight = vizContainer.clientHeight - (margin * 2);
            
            // Skala bazowa - cała wysokość wizualizacji = 30m (od -15m do +15m)
            const fullRangeMeters = 30; // -15m do +15m
            const baseScale = availableHeight / fullRangeMeters; // Skala dostosowana do wysokości kontenera
            
            // Skalowanie - stałe niezależnie od odległości
            let scale = baseScale;
            let vehicleScale = 1.0; // Stały rozmiar ikon
            
            
            // Pozycja sieczkarni w centrum z skalowaniem i offsetem
            const chopperWidth = BASE_CHOPPER_SIZE.width * CHOPPER_SCALE * vehicleScale;
            const chopperHeight = BASE_CHOPPER_SIZE.height * CHOPPER_SCALE * vehicleScale;
            chopperElement.style.left = `${containerCenterX - chopperWidth/2 + CHOPPER_OFFSET.x}px`;
            chopperElement.style.top = `${containerCenterY - chopperHeight/2 + CHOPPER_OFFSET.y}px`;
            
            // Skalowanie ikony sieczkarni - zachowuje rzeczywiste proporcje
            const chopperIcon = chopperElement.querySelector('.vehicle-icon');
            if (chopperIcon) {
                chopperIcon.style.transform = `scale(${vehicleScale})`;
            }
            
            // Pozycjonowanie punktu referencyjnego sieczkarni (punkt 0,0)
            const chopperRefPoint = document.getElementById('chopper-ref-point');
            if (chopperRefPoint && SHOW_REFERENCE_POINTS) {
                chopperRefPoint.style.left = `${containerCenterX}px`;
                chopperRefPoint.style.top = `${containerCenterY}px`;
            }
            
            // 2. Obliczenie heading względnego ciągnika względem sieczkarni
            let relativeHeading = 0;
            if (message.tractor_gps_filtered.heading_deg !== PLACEHOLDER_FLOAT && 
                message.chopper_gps.heading_deg !== PLACEHOLDER_FLOAT) {
                relativeHeading = message.tractor_gps_filtered.heading_deg - message.chopper_gps.heading_deg;
                // Normalizacja do zakresu -180 do 180 stopni
                while (relativeHeading > 180) relativeHeading -= 360;
                while (relativeHeading < -180) relativeHeading += 360;
            }
            
            // 3. Pozycja ciągnika względem sieczkarni (centrum)
            const tractorX = containerCenterX + (message.relative_position.distance_lateral * scale);
            const tractorY = containerCenterY - (message.relative_position.distance_longitudinal * scale);
            
            // Pozycjonowanie ciągnika z skalowaniem i offsetem
            const tractorWidth = BASE_TRACTOR_SIZE.width * TRACTOR_SCALE * vehicleScale;
            const tractorHeight = BASE_TRACTOR_SIZE.height * TRACTOR_SCALE * vehicleScale;
            tractorElement.style.left = `${tractorX - tractorWidth/2 + TRACTOR_OFFSET.x}px`;
            tractorElement.style.top = `${tractorY - tractorHeight/2 + TRACTOR_OFFSET.y}px`;
            
            // 4. Obracanie i skalowanie ciągnika - zachowuje rzeczywiste proporcje
            const tractorIcon = tractorElement.querySelector('.vehicle-icon');
            if (tractorIcon) {
                tractorIcon.style.transform = `rotate(${relativeHeading}deg) scale(${vehicleScale})`;
            }
            
            // Pozycjonowanie punktu referencyjnego ciągnika (punkt 0,0)
            const tractorRefPoint = document.getElementById('tractor-ref-point');
            if (tractorRefPoint && SHOW_REFERENCE_POINTS) {
                tractorRefPoint.style.left = `${tractorX}px`;
                tractorRefPoint.style.top = `${tractorY}px`;
            }
            
            // Pozycjonowanie pól prędkości bezpośrednio pod ikonami
            const chopperSpeedEl = document.getElementById('chopper-speed');
            if (chopperSpeedEl) {
                chopperSpeedEl.style.left = `${containerCenterX - chopperSpeedEl.offsetWidth/2}px`;
                chopperSpeedEl.style.top = `${containerCenterY + chopperHeight/2 + 5}px`;
            }
            
            const tractorSpeedEl = document.getElementById('tractor-speed');
            if (tractorSpeedEl) {
                tractorSpeedEl.style.left = `${tractorX - tractorSpeedEl.offsetWidth/2}px`;
                tractorSpeedEl.style.top = `${tractorY - tractorSpeedEl.offsetHeight/2 + 80}px`;
            }
        }
    }

    // --- Logika strojenia PID ---
    const kpSlider = document.getElementById('kp-slider');
    const kiSlider = document.getElementById('ki-slider');
    const kdSlider = document.getElementById('kd-slider');  // Dodany slider Kd
    const kpValueSpan = document.getElementById('kp-value');
    const kiValueSpan = document.getElementById('ki-value');
    const kdValueSpan = document.getElementById('kd-value');  // Dodany span Kd
    
    // --- Logika strojenia regulatora pozycji ---
    const positionKpSlider = document.getElementById('position-kp-slider');
    const positionKiSlider = document.getElementById('position-ki-slider');
    const positionToleranceSlider = document.getElementById('position-tolerance-slider');
    const speedToleranceSlider = document.getElementById('speed-tolerance-slider');
    const positionKpValueSpan = document.getElementById('position-kp-value');
    const positionKiValueSpan = document.getElementById('position-ki-value');
    const positionToleranceValueSpan = document.getElementById('position-tolerance-value');
    const speedToleranceValueSpan = document.getElementById('speed-tolerance-value');

    kpSlider.oninput = () => kpValueSpan.textContent = parseFloat(kpSlider.value).toFixed(1);
    kiSlider.oninput = () => kiValueSpan.textContent = parseFloat(kiSlider.value).toFixed(1);
    kdSlider.oninput = () => kdValueSpan.textContent = parseFloat(kdSlider.value).toFixed(1);
    
    // Slajdery regulatora pozycji
    if (positionKpSlider) {
        positionKpSlider.oninput = () => positionKpValueSpan.textContent = parseFloat(positionKpSlider.value).toFixed(1);
    }
    if (positionKiSlider) {
        positionKiSlider.oninput = () => positionKiValueSpan.textContent = parseFloat(positionKiSlider.value).toFixed(2);
    }
    if (positionToleranceSlider) {
        positionToleranceSlider.oninput = () => positionToleranceValueSpan.textContent = parseFloat(positionToleranceSlider.value).toFixed(1);
    }
    if (speedToleranceSlider) {
        speedToleranceSlider.oninput = () => speedToleranceValueSpan.textContent = parseFloat(speedToleranceSlider.value).toFixed(2);
    }
    
    // Serwis do ustawiania parametrów
    const setParamsClient = new ROSLIB.Service({
        ros: ros,
        name: '/speed_controller_node/set_parameters',
        serviceType: 'rcl_interfaces/srv/SetParameters'
    });

    document.getElementById('apply-pid-btn').onclick = () => {
        const params = [
            { name: 'kp', value: { type: 2, double_value: parseFloat(kpSlider.value) } },
            { name: 'ki', value: { type: 2, double_value: parseFloat(kiSlider.value) } },
            { name: 'kd', value: { type: 2, double_value: parseFloat(kdSlider.value) } }
        ];
        const request = new ROSLIB.ServiceRequest({ parameters: params });
        setParamsClient.callService(request, (result) => {
            if (result.results.every(r => r.successful)) {
                showNotification('Parametry zaktualizowane!', 'success');
            } else {
                showNotification('Błąd podczas aktualizacji parametrów.', 'error');
            }
            // Automatyczne zamknięcie okienka popup - zawsze
            if (modal) {
                modal.style.display = 'none';
            }
        });
    };

    // --- Serwis do ustawiania parametrów regulatora pozycji ---
    const setPositionParamsClient = new ROSLIB.Service({
        ros: ros,
        name: '/position_controller_node/set_parameters',
        serviceType: 'rcl_interfaces/srv/SetParameters'
    });

    const applyPositionPidBtn = document.getElementById('apply-position-pid-btn');
    if (applyPositionPidBtn) {
        applyPositionPidBtn.onclick = () => {
            const params = [
                { name: 'Kp', value: { type: 2, double_value: parseFloat(positionKpSlider.value) } },
                { name: 'Ki', value: { type: 2, double_value: parseFloat(positionKiSlider.value) } },
                { name: 'position_tolerance', value: { type: 2, double_value: parseFloat(positionToleranceSlider.value) } },
                { name: 'speed_tolerance', value: { type: 2, double_value: parseFloat(speedToleranceSlider.value) } }
            ];
            const request = new ROSLIB.ServiceRequest({ parameters: params });
            setPositionParamsClient.callService(request, (result) => {
                if (result.results.every(r => r.successful)) {
                    showNotification('Parametry regulatora pozycji zaktualizowane!', 'success');
                } else {
                    showNotification('Błąd aktualizacji parametrów regulatora pozycji!', 'error');
                }
                // Automatyczne zamknięcie okienka popup
                if (positionModal) {
                    positionModal.style.display = 'none';
                }
            });
        };
    }

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

    // --- NOWY: Serwis dla regulatora prędkości ---
    const setSpeedControllerClient = new ROSLIB.Service({
        ros: ros,
        name: '/speed_controller/set_enabled',
        serviceType: 'std_srvs/srv/SetBool'
    });

    // --- NOWY: Serwis dla regulatora pozycji ---
    const setPositionControllerClient = new ROSLIB.Service({
        ros: ros,
        name: '/position_controller/set_enabled',
        serviceType: 'std_srvs/srv/SetBool'
    });

    toggleAutopilotBtn.onclick = () => {
        const targetState = !isAutopilotOn;
        const request = new ROSLIB.ServiceRequest({ data: targetState });
        setAutopilotClient.callService(request, (result) => {
            if (result.success) {
                isAutopilotOn = targetState;
                
                // NOWA LOGIKA: Główny autopilot automatycznie włącza regulator prędkości
                if (isAutopilotOn) {
                    // Autopilot włączony - automatycznie włącz regulator prędkości
                    isSpeedControllerEnabled = true;
                    
                    // Aktualizuj UI regulatora prędkości
                    if (toggleSpeedControllerBtn) {
                        toggleSpeedControllerBtn.textContent = 'WŁĄCZONA';
                        toggleSpeedControllerBtn.className = 'btn-toggle btn-toggle-on';
                    }
                    
                    showNotification('Autopilot aktywowany - regulator prędkości automatycznie włączony', 'success');
                } else {
                    // Autopilot wyłączony - wyłącz wszystkie regulatory
                    isSpeedControllerEnabled = false;
                    
                    // Aktualizuj UI regulatora prędkości
                    if (toggleSpeedControllerBtn) {
                        toggleSpeedControllerBtn.textContent = 'WYŁĄCZONA';
                        toggleSpeedControllerBtn.className = 'btn-toggle btn-toggle-off';
                    }
                    
                    showNotification('Autopilot dezaktywowany - wszystkie regulatory wyłączone', 'warning');
                }
                
                updateAutopilotUI();
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
            
            // NOWA: Przycisk regulatora prędkości jest zablokowany gdy autopilot aktywny
            if (toggleSpeedControllerBtn) {
                toggleSpeedControllerBtn.className = 'btn-toggle btn-toggle-disabled';
                toggleSpeedControllerBtn.title = 'Autopilot aktywny - użyj głównego przycisku do wyłączenia';
            }
            
            // NOWA: Aktualizuj diodę LED - regulator włączony przez autopilot
            if (speedControllerLed) {
                speedControllerLed.className = 'status-led on';
            }
        } else {
            autopilotStatusDiv.className = 'status-indicator status-off';
            autopilotStatusDiv.textContent = 'AUTOPILOT WYŁĄCZONY';
            toggleAutopilotBtn.className = 'btn-engage';
            toggleAutopilotBtn.textContent = 'AKTYWUJ';
            
            // NOWA: Przycisk regulatora prędkości jest aktywny gdy autopilot wyłączony
            if (toggleSpeedControllerBtn) {
                // Przywróć normalny wygląd przycisku
                if (isSpeedControllerEnabled) {
                    toggleSpeedControllerBtn.className = 'btn-toggle btn-toggle-on';
                } else {
                    toggleSpeedControllerBtn.className = 'btn-toggle btn-toggle-off';
                }
                toggleSpeedControllerBtn.title = '';
            }
            
            // NOWA: Aktualizuj diodę LED - regulator wyłączony przez autopilot
            if (speedControllerLed) {
                speedControllerLed.className = 'status-led off';
            }
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
            // Sprawdź czy to element statystyk systemu MSS - jeśli tak, nie nadpisuj className
            if (id === 'active_nodes_count' || id === 'error_nodes_count' || id === 'warning_nodes_count') {
                el.textContent = value;
                // Zachowaj oryginalne klasy CSS
                return;
            }
            
            // Dla innych elementów - standardowa logika
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

    // --- NOWA SEKCJA: Logika zakładki Control ---
    
    // Klienci usług dla kontroli (setSpeedClient usunięty - używamy topiku)

    const gearShiftUpClient = new ROSLIB.Service({
        ros: ros,
        name: '/gear_shift_up',
        serviceType: 'std_srvs/srv/SetBool'
    });

    const gearShiftDownClient = new ROSLIB.Service({
        ros: ros,
        name: '/gear_shift_down',
        serviceType: 'std_srvs/srv/SetBool'
    });

    // NOWY: Klient dla sterowania serwem
    const setServoAngleClient = new ROSLIB.Service({
        ros: ros,
        name: '/servo/set_angle',
        serviceType: 'my_robot_interfaces/srv/SetServoAngle'
    });

    // Elementy UI zakładki Control
    const speedControllerLed = document.getElementById('speed-controller-led');
    const toggleSpeedControllerBtn = document.getElementById('toggle-speed-controller-btn');
    const gearManagerStatus = document.getElementById('gear-manager-status');
    const toggleGearManagerBtn = document.getElementById('toggle-gear-manager-btn');
    
    // Elementy UI regulatora pozycji
    const positionControllerLed = document.getElementById('position-controller-led');
    const togglePositionControllerBtn = document.getElementById('toggle-position-controller-btn');
    const targetPositionInput = document.getElementById('target-position-regulator');
    const setPositionBtn = document.getElementById('set-position-regulator-btn');
    const gearUpBtn = document.getElementById('gear-up-btn');
    const gearDownBtn = document.getElementById('gear-down-btn');
    const currentGearDisplay = document.getElementById('current-gear-display');
    const rosStatusIndicator = document.getElementById('ros-status-indicator');
    const lastCommand = document.getElementById('last-command');
    const servoStatusIndicator = document.getElementById('servo-status-indicator');

    // NOWE: Elementy UI dla sterowania serwem
    const servoStatusDisplay = document.getElementById('servo-status-display');
    const servoTargetInput = document.getElementById('servo-target-input');
    const setServoBtn = document.getElementById('set-servo-btn');
    const servoPositionDisplay = document.getElementById('servo-position-display');
    const servoLeftBtn = document.getElementById('servo-left-btn');
    const servoCenterBtn = document.getElementById('servo-center-btn');
    const servoRightBtn = document.getElementById('servo-right-btn');
    const toggleServoModeBtn = document.getElementById('toggle-servo-mode-btn');

    // NOWE: Elementy UI dla sterowania prędkością w zakładce Regulator
    const targetSpeedRegulator = document.getElementById('target-speed-regulator');
    const setSpeedRegulatorBtn = document.getElementById('set-speed-regulator-btn');
    const currentSpeedDisplay = document.getElementById('current-speed-display');

    // NOWY: Klient dla przełączania trybu serwa
    const setServoModeClient = new ROSLIB.Service({
        ros: ros,
        name: '/servo/set_manual_mode',
        serviceType: 'std_srvs/srv/SetBool'
    });

    // NOWA: Logika przełącznika trybu serwa
    let isServoManualMode = false;
    
    toggleServoModeBtn.onclick = () => {
        console.log('Kliknięto przełącznik trybu serwa');
        const targetMode = !isServoManualMode;
        console.log('Przełączam na tryb:', targetMode ? 'ręczny' : 'automatyczny');
        
        const request = new ROSLIB.ServiceRequest({ data: targetMode });
        console.log('Wywołuję service /servo/set_manual_mode z request:', request);
        
        setServoModeClient.callService(request, (result) => {
            console.log('Otrzymano odpowiedź service:', result);
            if (result.success) {
                isServoManualMode = targetMode;
                updateServoModeUI();
                showNotification(
                    isServoManualMode ? 'Tryb ręczny serwa włączony' : 'Tryb automatyczny serwa włączony', 
                    isServoManualMode ? 'success' : 'warning'
                );
                updateLastCommand(
                    isServoManualMode ? 'Serwo: tryb ręczny' : 'Serwo: tryb automatyczny'
                );
            } else {
                console.error('Service zwrócił błąd:', result);
                showNotification('Błąd podczas zmiany trybu serwa!', 'error');
            }
        });
    };

    function updateServoModeUI() {
        if (isServoManualMode) {
            toggleServoModeBtn.textContent = 'RĘCZNY';
            toggleServoModeBtn.className = 'btn-toggle btn-toggle-on';
            
            // Włącz kontrolki serwa
            servoTargetInput.disabled = false;
            setServoBtn.disabled = false;
            servoLeftBtn.disabled = false;
            servoCenterBtn.disabled = false;
            servoRightBtn.disabled = false;
        } else {
            toggleServoModeBtn.textContent = 'AUTOMATYCZNY';
            toggleServoModeBtn.className = 'btn-toggle btn-toggle-off';
            
            // Wyłącz kontrolki serwa
            servoTargetInput.disabled = true;
            setServoBtn.disabled = true;
            servoLeftBtn.disabled = true;
            servoCenterBtn.disabled = true;
            servoRightBtn.disabled = true;
        }
    }

    // Kontrola biegów
    gearUpBtn.onclick = () => {
        const request = new ROSLIB.ServiceRequest({ data: true });
        gearShiftUpClient.callService(request, (result) => {
            if (result.success) {
                showNotification('Zmieniono bieg w górę', 'success');
                updateLastCommand('Bieg w górę');
            } else {
                showNotification('Błąd podczas zmiany biegu!', 'error');
            }
        });
    };

    gearDownBtn.onclick = () => {
        const request = new ROSLIB.ServiceRequest({ data: true });
        gearShiftDownClient.callService(request, (result) => {
            if (result.success) {
                showNotification('Zmieniono bieg w dół', 'success');
                updateLastCommand('Bieg w dół');
            } else {
                showNotification('Błąd podczas zmiany biegu!', 'error');
            }
        });
    };

    // NOWA: Sterowanie prędkością z zakładki Regulator
    setSpeedRegulatorBtn.onclick = () => {
        const targetSpeed = parseFloat(targetSpeedRegulator.value);
        if (isNaN(targetSpeed) || targetSpeed < 0) {
            showNotification('Nieprawidłowa prędkość!', 'error');
            return;
        }

        // Publikuj na topik /target_speed (tak jak speed_teleop_node)
        const speedMsg = new ROSLIB.Message({
            data: targetSpeed
        });
        
        const speedPublisher = new ROSLIB.Topic({
            ros: ros,
            name: '/target_speed',
            messageType: 'std_msgs/msg/Float64'
        });
        
        speedPublisher.publish(speedMsg);
        showNotification(`Prędkość zadana ustawiona na ${targetSpeed} m/s`, 'success');
        updateLastCommand(`Ustaw prędkość: ${targetSpeed} m/s`);
    };

    // NOWA: Sterowanie serwem
    setServoBtn.onclick = () => {
        const targetAngle = parseInt(servoTargetInput.value);
        if (isNaN(targetAngle) || targetAngle < 0 || targetAngle > 180) {
            showNotification('Nieprawidłowy kąt! (0-180°)', 'error');
            return;
        }

        // Publikuj na topik /servo/set_angle
        const servoMsg = new ROSLIB.Message({
            data: targetAngle
        });
        
        // Używamy topiku zamiast service (zgodnie z istniejącą architekturą)
        const servoPublisher = new ROSLIB.Topic({
            ros: ros,
            name: '/servo/set_angle',
            messageType: 'my_robot_interfaces/msg/StampedInt32'
        });
        
        servoPublisher.publish(servoMsg);
        showNotification(`Serwo ustawione na ${targetAngle}°`, 'success');
        updateLastCommand(`Serwo: ${targetAngle}°`);
    };

    // NOWA: Szybkie pozycje serwa
    servoLeftBtn.onclick = () => {
        const servoPublisher = new ROSLIB.Topic({
            ros: ros,
            name: '/servo/set_angle',
            messageType: 'my_robot_interfaces/msg/StampedInt32'
        });
        
        const servoMsg = new ROSLIB.Message({
            data: 0
        });
        
        servoPublisher.publish(servoMsg);
        showNotification('Serwo ustawione na lewo (0°)', 'success');
        updateLastCommand('Serwo: lewo (0°)');
    };

    servoCenterBtn.onclick = () => {
        const servoPublisher = new ROSLIB.Topic({
            ros: ros,
            name: '/servo/set_angle',
            messageType: 'my_robot_interfaces/msg/StampedInt32'
        });
        
        const servoMsg = new ROSLIB.Message({
            data: 90
        });
        
        servoPublisher.publish(servoMsg);
        showNotification('Serwo ustawione na środek (90°)', 'success');
        updateLastCommand('Serwo: środek (90°)');
    };

    servoRightBtn.onclick = () => {
        const servoPublisher = new ROSLIB.Topic({
            ros: ros,
            name: '/servo/set_angle',
            messageType: 'my_robot_interfaces/msg/StampedInt32'
        });
        
        const servoMsg = new ROSLIB.Message({
            data: 180
        });
        
        servoPublisher.publish(servoMsg);
        showNotification('Serwo ustawione na prawo (180°)', 'success');
        updateLastCommand('Serwo: prawo (180°)');
    };

    // Aktualizacja statusu połączenia ROS
    ros.on('connection', () => {
        rosStatusIndicator.textContent = 'POŁĄCZONO';
        rosStatusIndicator.className = 'status-indicator-small connected';
    });
    
    ros.on('error', () => {
        rosStatusIndicator.textContent = 'BŁĄD';
        rosStatusIndicator.className = 'status-indicator-small disconnected';
    });
    
    ros.on('close', () => {
        rosStatusIndicator.textContent = 'ROZŁĄCZONO';
        rosStatusIndicator.className = 'status-indicator-small disconnected';
    });

    // Aktualizacja statusu regulatora na podstawie danych diagnostycznych
    diagnosticsListener.subscribe((message) => {
        // Aktualizacja statusu regulatora
        if (message.target_speed.data !== PLACEHOLDER_FLOAT) {
            speedControllerLed.className = 'status-led on';
        } else {
            speedControllerLed.className = 'status-led off';
        }

        // Status gear manager będzie aktualizowany przez subskrypcję health

        // Aktualizacja wyświetlacza biegu
        if (message.tractor_gear.gear !== 255) {
            currentGearDisplay.textContent = message.tractor_gear.gear;
        } else {
            currentGearDisplay.textContent = '---';
        }

        // Aktualizacja statusu serwa
        if (message.servo_position.data !== PLACEHOLDER_INT) {
            servoStatusIndicator.textContent = 'OK';
            servoStatusIndicator.className = 'status-indicator-small connected';
            
            // NOWE: Aktualizacja wyświetlacza pozycji serwa w zakładce Control
            servoPositionDisplay.textContent = `${message.servo_position.data}°`;
            servoStatusDisplay.textContent = 'OK';
            servoStatusDisplay.className = 'status-display active';
        } else {
            servoStatusIndicator.textContent = 'BŁĄD';
            servoStatusIndicator.className = 'status-indicator-small disconnected';
            
            // NOWE: Aktualizacja wyświetlacza pozycji serwa w zakładce Control
            servoPositionDisplay.textContent = '---°';
            servoStatusDisplay.textContent = 'BŁĄD';
            servoStatusDisplay.className = 'status-display inactive';
        }
    });

    // Funkcja aktualizacji ostatniej komendy
    function updateLastCommand(command) {
        const timestamp = new Date().toLocaleTimeString('pl-PL');
        lastCommand.textContent = `${timestamp}: ${command}`;
    }

    // Inicjalizacja statusów
    updateLastCommand('System gotowy');

    // NOWA: Inicjalizacja UI serwa
    updateServoModeUI();

    // NOWA: Inicjalizacja UI regulatora prędkości
    if (toggleSpeedControllerBtn) {
        toggleSpeedControllerBtn.textContent = 'WYŁĄCZONA';
        toggleSpeedControllerBtn.className = 'btn-toggle btn-toggle-off';
    }
    
    // NOWA: Inicjalizacja diody LED regulatora
    if (speedControllerLed) {
        speedControllerLed.className = 'status-led off';
    }

    // NOWA: Sprawdź aktualny stan regulatora prędkości na starcie
    checkSpeedControllerStatus();

    // NOWA: Funkcja do sprawdzania aktualnego stanu regulatora prędkości
    function checkSpeedControllerStatus() {
        // Sprawdź aktualny stan regulatora prędkości przez serwis
        const request = new ROSLIB.ServiceRequest({ data: false }); // false = sprawdź stan
        
        setSpeedControllerClient.callService(request, (result) => {
            // TODO: W przyszłości serwis powinien zwracać aktualny stan
            // Na razie zakładamy że regulator jest wyłączony na starcie
            isSpeedControllerEnabled = false;
            
            if (toggleSpeedControllerBtn) {
                toggleSpeedControllerBtn.textContent = 'WYŁĄCZONA';
                toggleSpeedControllerBtn.className = 'btn-toggle btn-toggle-off';
            }
            
            // NOWA: Aktualizuj diodę LED
            if (speedControllerLed) {
                speedControllerLed.className = 'status-led off';
            }
        });
    }

    // --- Logika przełącznika regulatora prędkości ---
    toggleSpeedControllerBtn.onclick = () => {
        if (isAutopilotOn) {
            // Gdy autopilot włączony - nie można ręcznie wyłączyć regulatora
            showNotification('Autopilot aktywny - użyj głównego przycisku autopilota do wyłączenia', 'warning');
            updateLastCommand('Próba ręcznego wyłączenia regulatora przy aktywnym autopilocie');
        } else {
            // Gdy autopilot wyłączony - można swobodnie włączać/wyłączać regulator
            const targetState = !isSpeedControllerEnabled;
            const request = new ROSLIB.ServiceRequest({ data: targetState });
            
            setSpeedControllerClient.callService(request, (result) => {
                if (result.success) {
                    isSpeedControllerEnabled = targetState;
                    
                    if (isSpeedControllerEnabled) {
                        toggleSpeedControllerBtn.textContent = 'WŁĄCZONA';
                        toggleSpeedControllerBtn.className = 'btn-toggle btn-toggle-on';
                        showNotification('Regulacja prędkości włączona', 'success');
                        updateLastCommand('Włączono regulację prędkości');
                        
                        // NOWA: Aktualizuj diodę LED
                        if (speedControllerLed) {
                            speedControllerLed.className = 'status-led on';
                        }
                    } else {
                        toggleSpeedControllerBtn.textContent = 'WYŁĄCZONA';
                        toggleSpeedControllerBtn.className = 'btn-toggle btn-toggle-off';
                        showNotification('Regulacja prędkości wyłączona', 'warning');
                        updateLastCommand('Wyłączono regulację prędkości');
                        
                        // NOWA: Aktualizuj diodę LED
                        if (speedControllerLed) {
                            speedControllerLed.className = 'status-led off';
                        }
                    }
                } else {
                    showNotification("Nie udało się zmienić stanu regulatora prędkości!", 'error');
                }
            });
        }
    };

    // --- NOWY: Klient serwisu gear manager ---
    const setGearManagerClient = new ROSLIB.Service({
        ros: ros,
        name: '/gear_manager/set_enabled',
        serviceType: 'std_srvs/srv/SetBool'
    });

    // --- Logika przełącznika gear manager ---
    let isGearManagerEnabled = false;
    
    toggleGearManagerBtn.onclick = () => {
        const targetState = !isGearManagerEnabled;
        const request = new ROSLIB.ServiceRequest({ data: targetState });
        
        setGearManagerClient.callService(request, (result) => {
            if (result.success) {
                isGearManagerEnabled = targetState;
                
                if (isGearManagerEnabled) {
                    toggleGearManagerBtn.textContent = 'WŁĄCZONE';
                    toggleGearManagerBtn.className = 'btn-toggle btn-toggle-on';
                    showNotification('Automatyczne zarządzanie biegami włączone', 'success');
                    updateLastCommand('Włączono gear manager');
                } else {
                    toggleGearManagerBtn.textContent = 'WYŁĄCZONE';
                    toggleGearManagerBtn.className = 'btn-toggle btn-toggle-off';
                    showNotification('Automatyczne zarządzanie biegami wyłączone', 'warning');
                    updateLastCommand('Wyłączono gear manager');
                }
            } else {
                showNotification("Nie udało się zmienić stanu gear managera!", 'error');
            }
        });
    };

    // --- Logika przełącznika regulatora pozycji ---
    if (togglePositionControllerBtn) {
        togglePositionControllerBtn.onclick = () => {
            const targetState = !isPositionControllerEnabled;
            const request = new ROSLIB.ServiceRequest({ data: targetState });
            
            setPositionControllerClient.callService(request, (result) => {
                if (result.success) {
                    isPositionControllerEnabled = targetState;
                    
                    if (isPositionControllerEnabled) {
                        togglePositionControllerBtn.textContent = 'WŁĄCZONY';
                        togglePositionControllerBtn.className = 'btn-toggle btn-toggle-on';
                        showNotification('Autopilot pozycji włączony', 'success');
                        updateLastCommand('Włączono autopilot pozycji');
                        
                        if (positionControllerLed) {
                            positionControllerLed.className = 'status-led on';
                        }
                    } else {
                        togglePositionControllerBtn.textContent = 'WYŁĄCZONY';
                        togglePositionControllerBtn.className = 'btn-toggle btn-toggle-off';
                        showNotification('Autopilot pozycji wyłączony', 'info');
                        updateLastCommand('Wyłączono autopilot pozycji');
                        
                        if (positionControllerLed) {
                            positionControllerLed.className = 'status-led off';
                        }
                    }
                } else {
                    showNotification('Błąd zmiany stanu autopilota pozycji', 'error');
                    updateLastCommand('Błąd zmiany stanu autopilota pozycji');
                }
            });
        };
    }

    // --- Logika ustawiania pozycji zadanej ---
    if (setPositionBtn && targetPositionInput) {
        setPositionBtn.onclick = () => {
            const targetPosition = parseFloat(targetPositionInput.value);
            if (!isNaN(targetPosition)) {
                // Ustawienie pozycji zadanej przez serwis
                const request = new ROSLIB.ServiceRequest({
                    parameters: [{
                        name: 'target_distance',
                        value: {
                            type: 'double',
                            double_value: targetPosition
                        }
                    }]
                });
                
                setPositionParamsClient.callService(request, (result) => {
                    if (result.successful) {
                        window.lastTargetPosition = targetPosition;
                        showNotification(`Ustawiono pozycję zadaną: ${targetPosition} m`, 'success');
                        updateLastCommand(`Ustawiono pozycję zadaną: ${targetPosition} m`);
                    } else {
                        showNotification('Błąd ustawiania pozycji zadanej', 'error');
                    }
                });
            } else {
                showNotification('Nieprawidłowa wartość pozycji', 'error');
            }
        };
    }
    // ====================================================
    
    // === NOWE SUBSKRYPCJE DIAGNOSTYCZNE ===
    
    // 1. System Status - ogólny status systemu MSS
    const systemStatusListener = new ROSLIB.Topic({
        ros: ros,
        name: '/mss/system_status',
        messageType: 'std_msgs/msg/String'
    });

    systemStatusListener.subscribe((message) => {
        try {
            const systemData = JSON.parse(message.data);
            
            // === NOWA FUNKCJA: Użycie updateOverallHealthStatus z dashboard ===
            updateOverallHealthStatus(systemData);
            
            // === STARA FUNKCJA: Zachowuję dla kompatybilności ===
            // Aktualizacja ogólnego statusu systemu
            updateText('overall_system_status', systemData.overall_status);
            updateInt('active_nodes_count', systemData.running_nodes);
            updateInt('error_nodes_count', systemData.error_nodes);
            updateText('health_last_update', new Date(systemData.timestamp).toLocaleString());
            
            // Aktualizacja statusu poszczególnych węzłów
            if (systemData.node_states) {
                updateNodeHealthStatus('gps_rtk_node', systemData.node_states.gps_rtk_node);
                updateNodeHealthStatus('bt_receiver_node', systemData.node_states.bt_receiver_node);
                updateNodeHealthStatus('gear_reader_node', systemData.node_states.gear_reader_node);
                updateNodeHealthStatus('servo_controller', systemData.node_states.servo_controller);
                updateNodeHealthStatus('gear_shifter', systemData.node_states.gear_shifter);
                updateNodeHealthStatus('speed_filter_node', systemData.node_states.speed_filter_node);
                updateNodeHealthStatus('speed_controller_node', systemData.node_states.speed_controller_node);
                updateNodeHealthStatus('relative_computer_node', systemData.node_states.relative_computer_node);
                updateNodeHealthStatus('gear_manager_node', systemData.node_states.gear_manager_node);
                updateNodeHealthStatus('diagnostics_node', systemData.node_states.diagnostics_node);
                updateNodeHealthStatus('system_monitor', systemData.node_states.system_monitor);
                updateNodeHealthStatus('mss_health_monitor_node', systemData.node_states.mss_health_monitor_node);
            }
            
        } catch (error) {
            console.error('Błąd parsowania system status:', error);
        }
    });

    // 2. Health Alerts - alerty o problemach
    const healthAlertsListener = new ROSLIB.Topic({
        ros: ros,
        name: '/mss/health_alerts',
        messageType: 'std_msgs/msg/String'
    });

    healthAlertsListener.subscribe((message) => {
        try {
            const alertData = JSON.parse(message.data);
            
            // Aktualizacja alertów
            updateText('last_alert', alertData.message);
            updateText('alert_level', alertData.level);
            updateText('alert_timestamp', new Date(alertData.timestamp).toLocaleString());
            
            // Zwiększ licznik alertów
            const currentCount = parseInt(document.getElementById('alerts_count').textContent) || 0;
            updateInt('alerts_count', currentCount + 1);
            
            // Pokaż powiadomienie
            showNotification(`Alert: ${alertData.message}`, alertData.level.toLowerCase());
            
        } catch (error) {
            console.error('Błąd parsowania health alerts:', error);
        }
    });

    // 3. System Monitor - monitoring RPi
    const systemMonitorListener = new ROSLIB.Topic({
        ros: ros,
        name: '/mss/node_health/system_monitor',
        messageType: 'std_msgs/msg/String'
    });

    systemMonitorListener.subscribe((message) => {
        try {
            const monitorData = JSON.parse(message.data);
            
            // === NOWA FUNKCJA: Użycie updateSystemMetrics z dashboard ===
            updateSystemMetrics(monitorData);
            
            // === STARA FUNKCJA: Zachowuję dla kompatybilności ===
            if (monitorData.metrics) {
                const metrics = monitorData.metrics;
                
                // Aktualizacja metryk systemu
                updateFloat('system_cpu', metrics.cpu_usage_percent, 1);
                updateFloat('system_ram', metrics.memory_usage_percent, 1);
                updateFloat('system_temp', metrics.temperature_celsius, 1);
                updateFloat('system_disk', metrics.disk_usage_percent, 1);
                
                // Konwersja uptime na godziny
                const uptimeHours = (metrics.uptime_seconds / 3600).toFixed(1);
                updateText('system_uptime', uptimeHours);
                
                // Status komponentów
                updateText('system_gpio', metrics.gpio_status);
                updateText('system_network', metrics.network_status);
                
                // Status USB/Serial
                if (typeof metrics.usb_serial_status === 'object') {
                    const usbStatus = `${metrics.usb_serial_status.usb_devices} USB, ${metrics.usb_serial_status.serial_ports.join(', ')}`;
                    updateText('system_usb_serial', usbStatus);
                } else {
                    updateText('system_usb_serial', metrics.usb_serial_status);
                }
                
                // Aktualizacja statusu połączeń na podstawie metryk
                updateConnectionStatus('gps_rtk_status', metrics.gpio_status === 'OK');
                updateConnectionStatus('bluetooth_status', metrics.network_status.includes('OK'));
                
                // Ostatnia aktualizacja
                updateText('last_update', new Date().toLocaleTimeString());
            }
            
        } catch (error) {
            console.error('Błąd parsowania system monitor:', error);
        }
    });

    // 4. Node Health - status poszczególnych węzłów
    const nodeHealthTopics = [
        'gps_rtk_node',
        'bt_receiver_node',
        'gear_reader_node',
        'servo_controller',
        'gear_shifter',
        'speed_filter_node',
        'speed_controller_node',
        'relative_computer_node',
        'gear_manager_node',
        'diagnostics_node',
        'system_monitor',
        'mss_health_monitor_node'
    ];

    // Tworzenie subskrypcji dla każdego węzła
    nodeHealthTopics.forEach(nodeName => {
        const topicName = `/mss/node_health/${nodeName}`;
        const listener = new ROSLIB.Topic({
            ros: ros,
            name: topicName,
            messageType: 'std_msgs/msg/String'
        });

        listener.subscribe((message) => {
            try {
                const healthData = JSON.parse(message.data);
                
                // === NOWA FUNKCJA: Użycie updateNodeHealthStatus z dashboard ===
                updateNodeHealthStatus(nodeName, healthData);
                
                // === STARA FUNKCJA: Zachowuję dla kompatybilności ===
                // Aktualizacja statusu węzła w czasie rzeczywistym
                updateNodeHealthRealTime(nodeName, healthData);
                
                // === NOWY: Specjalna obsługa gear_manager_node ===
                if (nodeName === 'gear_manager_node' && healthData.is_enabled !== undefined) {
                    isGearManagerEnabled = healthData.is_enabled;
                    
                    if (isGearManagerEnabled) {
                        toggleGearManagerBtn.textContent = 'WŁĄCZONE';
                        toggleGearManagerBtn.className = 'btn-toggle btn-toggle-on';
                        gearManagerStatus.textContent = 'AKTYWNY';
                        gearManagerStatus.className = 'status-display active';
                    } else {
                        toggleGearManagerBtn.textContent = 'WYŁĄCZONE';
                        toggleGearManagerBtn.className = 'btn-toggle btn-toggle-off';
                        gearManagerStatus.textContent = 'NIEAKTYWNY';
                        gearManagerStatus.className = 'status-display inactive';
                    }
                }
                
            } catch (error) {
                console.error(`Błąd parsowania health dla ${nodeName}:`, error);
            }
        });
    });

    // === FUNKCJE POMOCNICZE DLA DIAGNOSTYKI ===
    
    function updateNodeHealthStatus(elementId, status) {
        const element = document.getElementById(elementId);
        if (element) {
            element.textContent = status || 'UNKNOWN';
            element.className = 'detail-value';
            
            // Dodanie klasy CSS dla kolorowania
            if (status === 'running') {
                element.classList.add('value-ok');
            } else if (status === 'error' || status === 'timeout') {
                element.classList.add('value-bad');
            } else if (status === 'warning') {
                element.classList.add('value-warning');
            } else {
                element.classList.add('value-info');
            }
        }
    }
    
    function updateNodeHealthRealTime(nodeName, healthData) {
        // Aktualizacja w czasie rzeczywistym - można dodać dodatkowe funkcje
        // np. wykresy, logi, etc.
        console.log(`Health update for ${nodeName}:`, healthData);
    }
    
    function updateConnectionStatus(elementId, isConnected) {
        const element = document.getElementById(elementId);
        if (element) {
            if (isConnected) {
                element.textContent = 'OK';
                element.className = 'detail-value value-ok';
            } else {
                element.textContent = 'BŁĄD';
                element.className = 'detail-value value-bad';
            }
        }
    }
    
    // Inicjalizacja informacji o systemie
    function initializeSystemInfo() {
        // Informacje o ROS
        updateText('ros_version', 'ROS2 Humble');
        updateText('system_arch', 'ARM64 (Raspberry Pi)');
        updateText('rpi_model', 'Raspberry Pi 5');
        
        // Status ROS Bridge
        updateConnectionStatus('ros_bridge_status', ros.isConnected);
        
        // Nasłuchiwanie zmian połączenia ROS
        ros.on('connection', () => {
            updateConnectionStatus('ros_bridge_status', true);
            showNotification('Połączono z ROS Bridge', 'success');
        });
        
        ros.on('error', () => {
            updateConnectionStatus('ros_bridge_status', false);
            showNotification('Błąd połączenia z ROS Bridge', 'error');
        });
        
        ros.on('close', () => {
            updateConnectionStatus('ros_bridge_status', false);
            showNotification('Rozłączono z ROS Bridge', 'warning');
        });
    }
    
    // Inicjalizacja systemu diagnostycznego
    initializeSystemInfo();
    
    // === KONIEC SUBSKRYPCJI DIAGNOSTYCZNYCH ===

    // === NOWE FUNKCJE DLA DASHBOARD SYSTEM I HEALTH ===
    
    // Inicjalizacja dashboard (bez wykresów)
    let dashboardInitialized = false;
    
    // Aktualizacja metryk systemowych
    function updateSystemMetrics(healthData) {
        try {
            const metrics = healthData.metrics || {};
            
            // Aktualizacja wartości
            if (metrics.cpu_usage_percent !== undefined) {
                document.getElementById('system_cpu').textContent = metrics.cpu_usage_percent.toFixed(1);
            }
            
            if (metrics.memory_usage_percent !== undefined) {
                document.getElementById('system_ram').textContent = metrics.memory_usage_percent.toFixed(1);
            }
            
            if (metrics.temperature_celsius !== undefined) {
                document.getElementById('system_temp').textContent = metrics.temperature_celsius.toFixed(1);
            }
            
            if (metrics.disk_usage_percent !== undefined) {
                document.getElementById('system_disk').textContent = metrics.disk_usage_percent.toFixed(1);
            }
            
            if (metrics.uptime_seconds !== undefined) {
                const hours = Math.floor(metrics.uptime_seconds / 3600);
                document.getElementById('system_uptime').textContent = hours;
            }
            
            // Aktualizacja statusów
            if (metrics.gpio_status) {
                document.getElementById('system_gpio').textContent = metrics.gpio_status;
            }
            
            if (metrics.network_status) {
                document.getElementById('system_network').textContent = metrics.network_status;
            }
            
            if (metrics.usb_serial_status) {
                const status = metrics.usb_serial_status;
                if (typeof status === 'object') {
                    document.getElementById('system_usb_serial').textContent = 
                        `USB: ${status.usb_devices}, Serial: ${status.serial_ports.join(', ')}`;
                }
            }
            
        } catch (error) {
            console.error('Błąd aktualizacji metryk systemowych:', error);
        }
    }
    
    // Aktualizacja statusu health węzłów
    function updateNodeHealthStatus(nodeName, healthData) {
        try {
            // Sprawdź czy healthData to string (status) czy obiekt (pełne dane)
            let status = 'unknown';
            if (typeof healthData === 'string') {
                status = healthData;
            } else if (healthData && healthData.status) {
                status = healthData.status;
            }
            
            // Znajdź element statusu węzła
            const statusElement = document.getElementById(`node_${nodeName.replace('_node', '')}`);
            const indicatorElement = document.querySelector(`[data-node="${nodeName}"] .node-health-indicator`);
            
            if (statusElement) {
                statusElement.textContent = status;
            }
            
            if (indicatorElement) {
                indicatorElement.className = `node-health-indicator ${status}`;
            }
            
            // Aktualizacja konsoli logów tylko jeśli mamy pełne dane
            if (healthData && typeof healthData === 'object') {
                addLogMessage(nodeName, status, healthData);
            }
            
        } catch (error) {
            console.error(`Błąd aktualizacji statusu węzła ${nodeName}:`, error);
        }
    }
    
    // Dodawanie wiadomości do konsoli logów
    function addLogMessage(nodeName, status, healthData) {
        const consoleOutput = document.getElementById('console_output');
        if (!consoleOutput) return;
        
        const timestamp = new Date().toLocaleTimeString();
        const logLine = document.createElement('div');
        logLine.className = 'console-line';
        
        let message = `Węzeł ${nodeName}: ${status}`;
        let logClass = 'info';
        
        if (status === 'error') {
            logClass = 'error';
            if (healthData.errors && healthData.errors.length > 0) {
                message += ` - ${healthData.errors.join(', ')}`;
            }
        } else if (status === 'warning' || (healthData.warnings && healthData.warnings.length > 0)) {
            logClass = 'warning';
            if (healthData.warnings && healthData.warnings.length > 0) {
                message += ` - ${healthData.warnings.join(', ')}`;
            }
        }
        
        logLine.innerHTML = `
            <span class="log-timestamp">[${timestamp}]</span>
            <span class="log-message ${logClass}">${message}</span>
        `;
        
        consoleOutput.appendChild(logLine);
        
        // Ogranicz liczbę linii w konsoli
        while (consoleOutput.children.length > 100) {
            consoleOutput.removeChild(consoleOutput.firstChild);
        }
    }
    
    // Aktualizacja ogólnego statusu health
    function updateOverallHealthStatus(healthData) {
        try {
            const overallStatus = healthData.overall_status || 'UNKNOWN';
            const indicator = document.getElementById('overall_health_indicator');
            const indicatorDot = indicator.querySelector('.indicator-dot');
            const indicatorText = indicator.querySelector('.indicator-text');
            
            // Aktualizacja wskaźnika
            indicatorDot.className = 'indicator-dot';
            if (overallStatus === 'OK') {
                indicatorDot.style.background = '#27ae60';
                indicatorText.textContent = 'OK';
            } else if (overallStatus === 'WARNING') {
                indicatorDot.style.background = '#f39c12';
                indicatorText.textContent = 'OSTRZEŻENIE';
            } else if (overallStatus === 'ERROR') {
                indicatorDot.style.background = '#e74c3c';
                indicatorText.textContent = 'BŁĄD';
            } else {
                indicatorDot.style.background = '#95a5a6';
                indicatorText.textContent = 'UNKNOWN';
            }
            
            // Aktualizacja statystyk
            if (healthData.total_nodes !== undefined) {
                updateInt('active_nodes_count', healthData.running_nodes || 0);
                updateInt('error_nodes_count', healthData.error_nodes || 0);
                updateInt('warning_nodes_count', 
                    (healthData.total_nodes - (healthData.running_nodes || 0) - (healthData.error_nodes || 0)) || 0);
            }
            
        } catch (error) {
            console.error('Błąd aktualizacji ogólnego statusu health:', error);
        }
    }
    
    // Inicjalizacja kontrolek konsoli
    function initializeConsoleControls() {
        const clearLogsBtn = document.getElementById('clear-logs-btn');
        const exportLogsBtn = document.getElementById('export-logs-btn');
        const logLevelFilter = document.getElementById('log-level-filter');
        
        if (clearLogsBtn) {
            clearLogsBtn.addEventListener('click', () => {
                const consoleOutput = document.getElementById('console_output');
                if (consoleOutput) {
                    consoleOutput.innerHTML = `
                        <div class="console-line">
                            <span class="log-timestamp">[System]</span>
                            <span class="log-message">Konsola logów wyczyszczona...</span>
                        </div>
                    `;
                }
            });
        }
        
        if (exportLogsBtn) {
            exportLogsBtn.addEventListener('click', () => {
                const consoleOutput = document.getElementById('console_output');
                if (consoleOutput) {
                    const logs = Array.from(consoleOutput.children).map(line => {
                        const timestamp = line.querySelector('.log-timestamp').textContent;
                        const message = line.querySelector('.log-message').textContent;
                        return `${timestamp} ${message}`;
                    }).join('\n');
                    
                    const blob = new Blob([logs], { type: 'text/plain' });
                    const url = URL.createObjectURL(blob);
                    const a = document.createElement('a');
                    a.href = url;
                    a.download = `mss_logs_${new Date().toISOString().slice(0,19).replace(/:/g,'-')}.txt`;
                    a.click();
                    URL.revokeObjectURL(url);
                }
            });
        }
        
        if (logLevelFilter) {
            logLevelFilter.addEventListener('change', (e) => {
                const selectedLevel = e.target.value;
                const consoleOutput = document.getElementById('console_output');
                
                if (consoleOutput && selectedLevel !== 'all') {
                    Array.from(consoleOutput.children).forEach(line => {
                        const message = line.querySelector('.log-message');
                        if (message.classList.contains(selectedLevel)) {
                            line.style.display = 'block';
                        } else {
                            line.style.display = 'none';
                        }
                    });
                } else if (consoleOutput) {
                    Array.from(consoleOutput.children).forEach(line => {
                        line.style.display = 'block';
                    });
                }
            });
        }
    }
    
    // Inicjalizacja dashboard po załadowaniu DOM
    function initializeDashboard() {
        if (dashboardInitialized) return;
        
        initializeConsoleControls();
        
        // Dodaj pierwsze logi
        addLogMessage('System', 'info', { message: 'Dashboard zainicjalizowany' });
        
        dashboardInitialized = true;
    }
    
    // Wywołaj inicjalizację dashboard
    initializeDashboard();
});
