// === Rover Status Indicator Logic ===

// 1. Update the status indicator in the DOM
window.setRoverStatus = function(status) {
  const indicator = document.getElementById('rover-status-indicator');
  const text = document.getElementById('rover-status-text');
  if (!indicator || !text) return;

  if (status === 'connected') {
    indicator.style.background = '#22c55e'; // Green
    text.textContent = 'Rover Connected';
    text.style.color = '#22c55e';
  } else if (status === 'closed') {
    indicator.style.background = '#f59e42'; // Orange
    text.textContent = 'Rover Disconnected';
    text.style.color = '#f59e42';
  } else {
    indicator.style.background = '#f59e42'; // Orange (waiting)
    text.textContent = 'Waiting for rover...';
    text.style.color = '#f59e42';
  }
};

window.setRoverStatus(); // initialize

// 2. Setup the CAN bus heartbeat monitoring
window.setupCanbusHeartbeat = function(ros) {
  // Prevent duplicate subscriptions
  if (window._canbusHeartbeatSetup) return;
  window._canbusHeartbeatSetup = true;

  let lastCanbusRx = null;
  const ROVER_TIMEOUT_MS = 3000;

  // Create the subscriber
  const canbusSubscriber = new ROSLIB.Topic({
    ros: ros,
    name: '/canbus/rx',
    messageType: 'can_msgs/msg/Frame'
  });

  // Debug: confirm we're attempting to subscribe
  console.log("[RoverStatus] Subscribing to /canbus/rx...");

  canbusSubscriber.subscribe(function(message) {
    lastCanbusRx = Date.now();
    window.setRoverStatus('connected');
    // Debug: show message receipt
    // console.log("[RoverStatus] Received CANBUS RX message:", message);
  });

  // Timer to check for heartbeat timeout
  setInterval(() => {
    if (!lastCanbusRx || (Date.now() - lastCanbusRx > ROVER_TIMEOUT_MS)) {
      window.setRoverStatus('closed');
    }
  }, 500);

  // Clean up on ROS disconnect
  if (ros && typeof ros.on === 'function') {
    ros.on('close', () => {
      window.setRoverStatus('closed');
      // Optionally: canbusSubscriber.unsubscribe();
      lastCanbusRx = null;
      window._canbusHeartbeatSetup = false;
      console.log("[RoverStatus] ROS connection closed, status reset.");
    });
  }

  console.log("[RoverStatus] Subscribed to /canbus/rx for rover status.");
};

// 3. Auto-setup after ROS connects (safe for multiple calls)
if (window.ros) {
  if (typeof window.ros.on === 'function') {
    window.ros.on('connection', function() {
      window.setupCanbusHeartbeat(window.ros);
    });
  } else {
    // If ros is already connected, call directly
    window.setupCanbusHeartbeat(window.ros);
  }
}
