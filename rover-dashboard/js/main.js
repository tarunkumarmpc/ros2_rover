// Ensure all code runs after DOM is fully loaded
document.addEventListener('DOMContentLoaded', function() {
  // --- Debug Toggle ---
  const debugToggle = document.getElementById('debug-toggle');
  const body = document.body;
  let debugMode = false;
  if (debugToggle) {
    debugToggle.addEventListener('click', () => {
      debugMode = !debugMode;
      debugToggle.classList.toggle('active', debugMode);
      debugToggle.textContent = debugMode ? 'Debug ON' : 'Debug Mode';
      body.classList.toggle('debug-mode', debugMode);
    });
  }

  // --- Control Mode Toggle ---
  const controlToggle = document.getElementById('control-toggle');
  const controlLayout = document.getElementById('control-layout');
  const dashboard = document.getElementById('dashboard');
  let controlMode = false;
  if (controlToggle && controlLayout && dashboard) {
    controlToggle.addEventListener('click', () => {
      controlMode = !controlMode;
      controlToggle.classList.toggle('active', controlMode);
      controlToggle.textContent = controlMode ? 'Control Mode ON' : 'Control Mode';
      if (controlMode) {
        controlLayout.classList.remove('hidden');
        dashboard.style.display = 'none';
      } else {
        controlLayout.classList.add('hidden');
        dashboard.style.display = '';
      }
    });
  }

  // --- Dummy Action Button: Return to Previous Screen ---
  // Make sure your Dummy Action button has id="dummy-action-btn"
  const dummyActionBtn = document.getElementById('dummy-action-btn') || (controlLayout ? controlLayout.querySelector('button') : null);
  if (dummyActionBtn && controlLayout && dashboard && controlToggle) {
    dummyActionBtn.addEventListener('click', () => {
      controlLayout.classList.add('hidden');
      dashboard.style.display = '';
      controlMode = false;
      controlToggle.classList.remove('active');
      controlToggle.textContent = 'Control Mode';
    });
  }

  // --- Mode Buttons (unchanged) ---
  const manualBtn = document.getElementById('manual-btn');
  const autoBtn = document.getElementById('auto-btn');
  const modeCurrent = document.getElementById('mode-current');
  if (manualBtn && autoBtn && modeCurrent) {
    manualBtn.addEventListener('click', () => {
      manualBtn.classList.add('active');
      autoBtn.classList.remove('active');
      modeCurrent.textContent = "Manual";
      if (window.publishRoverMode) window.publishRoverMode("Manual");
    });
    autoBtn.addEventListener('click', () => {
      autoBtn.classList.add('active');
      manualBtn.classList.remove('active');
      modeCurrent.textContent = "Autonomous";
      if (window.publishRoverMode) window.publishRoverMode("Autonomous");
    });
  }

  // --- ROS Dashboard Initialization (unchanged) ---
  window.initializeDashboard = function() {
  window.ros = new ROSLIB.Ros({ url : 'ws://localhost:9090'});
    setRosStatus('connecting');
    window.ros.on('connection', () => {
      setRosStatus('connected');
      window.logActivity("Connected to ROS.");
      window.setupCameraFeed(window.ros);
      window.setupRosoutLog(window.ros);
      window.setupClientList(window.ros);
      window.setupJoystick(window.ros);
      window.setupRoverModePublisher(window.ros);
      window.setupLiftControls(window.ros);
      window.setupCanbusHeartbeat(window.ros);
      window.publishRoverMode(modeCurrent ? modeCurrent.textContent : "Manual");

    });
    window.ros.on('error', (err) => {
      setRosStatus('error');
      window.logActivity("ROS connection error: " + err);
    });
    window.ros.on('close', () => {
      setRosStatus('closed');
      window.logActivity("Disconnected from ROS.");
    });
  };
});

