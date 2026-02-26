/**
 * LLM PID Tuner Dashboard - Main Application
 *
 * This application connects to ROS via rosbridge and provides
 * a web interface for PID auto-tuning.
 */

// ============================================================================
// Configuration
// ============================================================================

const CONFIG = {
    rosbridgeUrl: 'ws://localhost:9090',
    topics: {
        tuningStatus: '/pid_tuner/tuning_status',
        tuningLog: '/pid_tuner/tuning_log',
        paramsUpdate: '/pid_tuner/params_update'
    },
    services: {
        startTuning: '/pid_tuner/start_tuning',
        stopTuning: '/pid_tuner/stop_tuning',
        setParams: '/pid_tuner/set_params'
    }
};

// ============================================================================
// Global State
// ============================================================================

let ros = null;
let isConnected = false;

// ROS Topics
let statusSubscriber = null;
let logSubscriber = null;

// ROS Service Clients
let startTuningClient = null;
let stopTuningClient = null;
let setParamsClient = null;

// Charts
let errorChart = null;
let paramsChart = null;

// Data history
let errorHistory = [];
let setpointHistory = [];
let paramsHistory = [];

// ============================================================================
// Initialization
// ============================================================================

document.addEventListener('DOMContentLoaded', () => {
    initCharts();
    initEventListeners();
    connectToROS();
});

function initCharts() {
    // Error Chart
    errorChart = echarts.init(document.getElementById('error-chart'));
    errorChart.setOption({
        tooltip: { trigger: 'axis' },
        legend: { data: ['误差', '设定值'], textStyle: { color: '#fff' } },
        xAxis: { type: 'category', data: [], axisLine: { lineStyle: { color: '#888' } } },
        yAxis: { type: 'value', axisLine: { lineStyle: { color: '#888' } }, splitLine: { lineStyle: { color: '#333' } } },
        series: [
            { name: '误差', type: 'line', data: [], smooth: true, lineStyle: { color: '#3498db' } },
            { name: '设定值', type: 'line', data: [], smooth: true, lineStyle: { color: '#2ecc71' } }
        ]
    });

    // Parameters Chart
    paramsChart = echarts.init(document.getElementById('params-chart'));
    paramsChart.setOption({
        tooltip: { trigger: 'axis' },
        legend: { data: ['Kp', 'Ki', 'Kd'], textStyle: { color: '#fff' } },
        xAxis: { type: 'category', data: [], axisLine: { lineStyle: { color: '#888' } } },
        yAxis: { type: 'value', axisLine: { lineStyle: { color: '#888' } }, splitLine: { lineStyle: { color: '#333' } } },
        series: [
            { name: 'Kp', type: 'line', data: [], smooth: true, lineStyle: { color: '#e74c3c' } },
            { name: 'Ki', type: 'line', data: [], smooth: true, lineStyle: { color: '#f39c12' } },
            { name: 'Kd', type: 'line', data: [], smooth: true, lineStyle: { color: '#9b59b6' } }
        ]
    });
}

function initEventListeners() {
    // Slider listeners
    document.getElementById('kp-slider').addEventListener('input', (e) => {
        document.getElementById('kp-value').textContent = e.target.value;
    });
    document.getElementById('ki-slider').addEventListener('input', (e) => {
        document.getElementById('ki-value').textContent = e.target.value;
    });
    document.getElementById('kd-slider').addEventListener('input', (e) => {
        document.getElementById('kd-value').textContent = e.target.value;
    });

    // Button listeners
    document.getElementById('apply-params-btn').addEventListener('click', applyParams);
    document.getElementById('start-tuning-btn').addEventListener('click', startTuning);
    document.getElementById('stop-tuning-btn').addEventListener('click', stopTuning);

    // Controller selection
    document.getElementById('controller-select').addEventListener('change', (e) => {
        updateJointOptions(e.target.value);
    });
}

// ============================================================================
// ROS Connection
// ============================================================================

function connectToROS() {
    addLog('正在连接到 ROS...', 'info');

    ros = new ROSLIB.Ros({ url: CONFIG.rosbridgeUrl });

    ros.on('connection', () => {
        isConnected = true;
        updateConnectionStatus(true);
        addLog('已连接到 ROS', 'success');

        // Subscribe to topics
        subscribeToTopics();

        // Create service clients
        createServiceClients();
    });

    ros.on('error', (error) => {
        addLog('ROS 连接错误: ' + error, 'error');
        updateConnectionStatus(false);
    });

    ros.on('close', () => {
        isConnected = false;
        updateConnectionStatus(false);
        addLog('ROS 连接已关闭', 'warning');

        // Try to reconnect after 5 seconds
        setTimeout(connectToROS, 5000);
    });
}

function updateConnectionStatus(connected) {
    const statusDot = document.querySelector('.status-dot');
    const statusText = document.querySelector('.status-text');

    if (connected) {
        statusDot.classList.add('connected');
        statusText.textContent = '已连接';
    } else {
        statusDot.classList.remove('connected');
        statusText.textContent = '未连接';
    }
}

// ============================================================================
// ROS Topics
// ============================================================================

function subscribeToTopics() {
    // Tuning Status
    statusSubscriber = new ROSLIB.Topic({
        ros: ros,
        name: CONFIG.topics.tuningStatus,
        messageType: 'rm_pid_tuner/TuningStatus'
    });
    statusSubscriber.subscribe(handleStatusMessage);

    // Tuning Log
    logSubscriber = new ROSLIB.Topic({
        ros: ros,
        name: CONFIG.topics.tuningLog,
        messageType: 'rm_pid_tuner/TuningLog'
    });
    logSubscriber.subscribe(handleLogMessage);
}

function handleStatusMessage(message) {
    // Update status display
    document.getElementById('tuning-status').textContent = message.status;
    document.getElementById('round-info').textContent = `${message.round} / ${message.max_rounds}`;
    document.getElementById('avg-error').textContent = message.avg_error.toFixed(2);
    document.getElementById('max-error').textContent = message.max_error.toFixed(2);

    // Update charts
    updateErrorChart(message);

    // Update PID sliders
    if (message.current_params) {
        document.getElementById('kp-slider').value = message.current_params.p;
        document.getElementById('kp-value').textContent = message.current_params.p.toFixed(2);
        document.getElementById('ki-slider').value = message.current_params.i;
        document.getElementById('ki-value').textContent = message.current_params.i.toFixed(2);
        document.getElementById('kd-slider').value = message.current_params.d;
        document.getElementById('kd-value').textContent = message.current_params.d.toFixed(2);
    }

    // Update button states
    const isTuning = message.status === 'TUNING';
    document.getElementById('start-tuning-btn').disabled = isTuning;
    document.getElementById('stop-tuning-btn').disabled = !isTuning;
}

function handleLogMessage(message) {
    const logText = `[第${message.round}轮] ${message.analysis} | 误差: ${message.avg_error.toFixed(2)}`;
    addLog(logText, 'info');

    // Update parameters history chart
    updateParamsChart(message);
}

// ============================================================================
// ROS Services
// ============================================================================

function createServiceClients() {
    startTuningClient = new ROSLIB.Service({
        ros: ros,
        name: CONFIG.services.startTuning,
        serviceType: 'rm_pid_tuner/StartTuning'
    });

    stopTuningClient = new ROSLIB.Service({
        ros: ros,
        name: CONFIG.services.stopTuning,
        serviceType: 'rm_pid_tuner/StopTuning'
    });

    setParamsClient = new ROSLIB.Service({
        ros: ros,
        name: CONFIG.services.setParams,
        serviceType: 'rm_pid_tuner/SetPidParams'
    });
}

function startTuning() {
    if (!isConnected) {
        addLog('未连接到 ROS', 'error');
        return;
    }

    const controllerName = document.getElementById('controller-select').value;
    const jointName = document.getElementById('joint-select').value;
    const targetSetpoint = parseFloat(document.getElementById('target-setpoint').value);
    const maxRounds = parseInt(document.getElementById('max-rounds').value);
    const conservativeMode = document.getElementById('conservative-mode').checked;

    const request = new ROSLIB.ServiceRequest({
        controller_name: controllerName,
        joint_names: [jointName],
        target_setpoint: targetSetpoint,
        tolerance: 0.3,
        max_rounds: maxRounds,
        buffer_size: 25,
        conservative_mode: conservativeMode,
        z_n_gain_factor: 0.5,
        auto_save: false
    });

    addLog(`开始调参: ${controllerName}/${jointName}`, 'info');

    startTuningClient.callService(request, (result) => {
        if (result.success) {
            addLog('调参已启动', 'success');
        } else {
            addLog('启动失败: ' + result.message, 'error');
        }
    });
}

function stopTuning() {
    if (!isConnected) {
        addLog('未连接到 ROS', 'error');
        return;
    }

    const request = new ROSLIB.ServiceRequest({
        controller_name: document.getElementById('controller-select').value,
        save_params: false,
        restore_params: false
    });

    stopTuningClient.callService(request, (result) => {
        if (result.success) {
            addLog(`调参已停止，共 ${result.total_rounds} 轮`, 'success');
        } else {
            addLog('停止失败: ' + result.message, 'error');
        }
    });
}

function applyParams() {
    if (!isConnected) {
        addLog('未连接到 ROS', 'error');
        return;
    }

    const p = parseFloat(document.getElementById('kp-slider').value);
    const i = parseFloat(document.getElementById('ki-slider').value);
    const d = parseFloat(document.getElementById('kd-slider').value);

    const request = new ROSLIB.ServiceRequest({
        controller_name: document.getElementById('controller-select').value,
        params: [{
            controller_name: document.getElementById('controller-select').value,
            joint_name: document.getElementById('joint-select').value,
            p: p,
            i: i,
            d: d,
            i_clamp_max: 0.3,
            i_clamp_min: -0.3,
            antiwindup: true,
            feedforward: 0.0
        }],
        apply_immediately: true,
        save_to_file: false
    });

    setParamsClient.callService(request, (result) => {
        if (result.success) {
            addLog(`参数已应用: Kp=${p}, Ki=${i}, Kd=${d}`, 'success');
        } else {
            addLog('应用失败: ' + result.message, 'error');
        }
    });
}

// ============================================================================
// Helper Functions
// ============================================================================

function updateJointOptions(controllerName) {
    const jointSelect = document.getElementById('joint-select');
    jointSelect.innerHTML = '';

    if (controllerName === 'gimbal_controller') {
        jointSelect.innerHTML = `
            <option value="yaw">Yaw</option>
            <option value="pitch">Pitch</option>
        `;
    } else if (controllerName === 'chassis_controller') {
        jointSelect.innerHTML = `
            <option value="follow">Follow</option>
        `;
    }
}

function updateErrorChart(message) {
    // Get current data
    const xData = errorChart.getOption().xAxis[0].data;
    const errorData = errorChart.getOption().series[0].data;
    const setpointData = errorChart.getOption().series[1].data;

    // Add new data point
    xData.push(message.round);
    errorData.push(message.avg_error);
    setpointData.push(message.target_setpoint || 0);

    // Keep only last 50 points
    if (xData.length > 50) {
        xData.shift();
        errorData.shift();
        setpointData.shift();
    }

    // Update chart
    errorChart.setOption({
        xAxis: { data: xData },
        series: [
            { data: errorData },
            { data: setpointData }
        ]
    });
}

function updateParamsChart(message) {
    const xData = paramsChart.getOption().xAxis[0].data;
    const pData = paramsChart.getOption().series[0].data;
    const iData = paramsChart.getOption().series[1].data;
    const dData = paramsChart.getOption().series[2].data;

    xData.push(message.round);
    pData.push(message.new_params.p);
    iData.push(message.new_params.i);
    dData.push(message.new_params.d);

    if (xData.length > 50) {
        xData.shift();
        pData.shift();
        iData.shift();
        dData.shift();
    }

    paramsChart.setOption({
        xAxis: { data: xData },
        series: [
            { data: pData },
            { data: iData },
            { data: dData }
        ]
    });
}

function addLog(message, type = 'info') {
    const logContainer = document.getElementById('log-container');
    const timestamp = new Date().toLocaleTimeString();

    const logEntry = document.createElement('div');
    logEntry.className = `log-entry ${type}`;
    logEntry.textContent = `[${timestamp}] ${message}`;

    logContainer.appendChild(logEntry);
    logContainer.scrollTop = logContainer.scrollHeight;
}
