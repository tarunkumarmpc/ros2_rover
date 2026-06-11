// lift_control.js

window.setupLiftControls = function(rosInstance) {
  // --- Publishers ---
  const roverLiftPublisher = new ROSLIB.Topic({
    ros: rosInstance,
    name: '/rover/rover_lift/target_height',
    messageType: 'std_msgs/Float64'
  });

  const cameraLiftPublisher = new ROSLIB.Topic({
    ros: rosInstance,
    name: '/rover/camera_lift/target_height',
    messageType: 'std_msgs/Float32'
  });

  // --- Rover Lift Controls ---
  const roverSlider = document.getElementById('rover-lift-slider');
  const roverTarget = document.getElementById('rover-lift-target');
  const roverSendBtn = document.getElementById('rover-lift-send-btn');
  const roverToggleBtn = document.getElementById('rover-lift-toggle-btn');
  const avgBlock = document.getElementById('rover-lift-actual-avg-block');
  const allBlock = document.getElementById('rover-lift-actual-all-block');
  const roverActualAvg = document.getElementById('rover-lift-actual-avg');
  const roverActualFL = document.getElementById('rover-lift-actual-fl');
  const roverActualFR = document.getElementById('rover-lift-actual-fr');
  const roverActualRL = document.getElementById('rover-lift-actual-rl');
  const roverActualRR = document.getElementById('rover-lift-actual-rr');

  if (roverSlider && roverTarget && roverSendBtn) {
    roverSlider.addEventListener('input', () => {
      roverTarget.textContent = parseFloat(roverSlider.value).toFixed(3);
    });

    roverSendBtn.addEventListener('click', () => {
      const value = parseFloat(roverSlider.value);
      roverLiftPublisher.publish({ data: value });
      if (window.logActivity) window.logActivity(`Rover lift target set to ${value.toFixed(3)} m`);
    });
  }

  // --- Toggle Button Logic for Rover Lift ---
  if (roverToggleBtn && avgBlock && allBlock) {
    roverToggleBtn.addEventListener('click', () => {
      const showingAll = !allBlock.classList.contains('hidden');
      if (showingAll) {
        allBlock.classList.add('hidden');
        avgBlock.classList.remove('hidden');
        roverToggleBtn.textContent = 'Show All';
      } else {
        allBlock.classList.remove('hidden');
        avgBlock.classList.add('hidden');
        roverToggleBtn.textContent = 'Show Average';
      }
    });
  }

  // --- ROS2 Subscription for Rover Lift Actual Values ---
  // Assumes topic is /rover/rover_lift/status, type rover_msgs/msg/RoverLiftStatus
  if (
    roverActualAvg &&
    roverActualFL &&
    roverActualFR &&
    roverActualRL &&
    roverActualRR
  ) {
    const roverLiftActualSub = new ROSLIB.Topic({
      ros: rosInstance,
      name: '/rover/rover_lift/status',
      messageType: 'rover_msgs/msg/RoverLiftStatus'
    });

    roverLiftActualSub.subscribe(function (msg) {
      // msg.data is an array of LiftData objects: {id, height, ...}
      // We'll sort by id to ensure FL, FR, RL, RR order (assuming ids 0-3)
      const dataArr = (msg.data || []).slice().sort((a, b) => a.id - b.id);
      // Defensive: if less than 4, fill with undefined
      const [fl, fr, rl, rr] = [
        dataArr[0] ? dataArr[0].height : undefined,
        dataArr[1] ? dataArr[1].height : undefined,
        dataArr[2] ? dataArr[2].height : undefined,
        dataArr[3] ? dataArr[3].height : undefined
      ];
      roverActualFL.textContent = fl !== undefined ? fl.toFixed(3) : '--';
      roverActualFR.textContent = fr !== undefined ? fr.toFixed(3) : '--';
      roverActualRL.textContent = rl !== undefined ? rl.toFixed(3) : '--';
      roverActualRR.textContent = rr !== undefined ? rr.toFixed(3) : '--';
      const avg =
        [fl, fr, rl, rr].every((v) => typeof v === 'number')
          ? (fl + fr + rl + rr) / 4
          : NaN;
      roverActualAvg.textContent =
        !isNaN(avg) && isFinite(avg) ? avg.toFixed(3) : '--';
    });
  }

  // --- Camera Lift Controls ---
  const cameraSlider = document.getElementById('camera-lift-slider');
  const cameraTarget = document.getElementById('camera-lift-target');
  const cameraSendBtn = document.getElementById('camera-lift-send-btn');
  const cameraActual = document.getElementById('camera-lift-actual');

  if (cameraSlider && cameraTarget && cameraSendBtn) {
    cameraSlider.addEventListener('input', () => {
      cameraTarget.textContent = parseFloat(cameraSlider.value).toFixed(3);
    });

    cameraSendBtn.addEventListener('click', () => {
      const value = parseFloat(cameraSlider.value);
      cameraLiftPublisher.publish({ data: value });
      if (window.logActivity) window.logActivity(`Camera lift target set to ${value.toFixed(3)} m`);
    });
  }

  // --- ROS2 Subscription for Camera Lift Actual Value ---
  // Uses /rover/camera_lift/status with custom message, height in mm
  if (cameraActual) {
    const cameraLiftActualSub = new ROSLIB.Topic({
      ros: rosInstance,
      name: '/rover/camera_lift/status',
      messageType: 'rover_msgs/msg/CameraLiftStatus'
    });
    cameraLiftActualSub.subscribe(function (msg) {
      cameraActual.textContent =
        typeof msg.height === 'number' ? (msg.height / 1000).toFixed(3) : '--';
    });
  }
};
