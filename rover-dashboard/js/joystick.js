window.setupJoystick = function(rosInstance) {
  // UI elements
  const velocityScaleSlider = document.getElementById('velocity-scale');
  const velocityScaleValue = document.getElementById('velocity-scale-value');
  const zone = document.getElementById('joystick-zone');
  const status = document.getElementById('joystick-status');

  // Initialize velocity scale
  let velocityScale = parseFloat(velocityScaleSlider.value);
  velocityScaleValue.textContent = velocityScale.toFixed(2);

  // Update velocity scale on slider input
  velocityScaleSlider.addEventListener('input', function() {
    velocityScale = parseFloat(this.value);
    velocityScaleValue.textContent = velocityScale.toFixed(2);
    console.log('[DEBUG] Velocity scale set to', velocityScale);
  });

  // ROS topic for velocity commands
  const cmdVel = new ROSLIB.Topic({
    ros: rosInstance,
    name: "/cmd_vel",
    messageType: "geometry_msgs/Twist"
  });

  // Initialize nipplejs joystick
  const joystick = nipplejs.create({
    zone: zone,
    mode: 'static',
    position: { left: '50%', top: '50%' },
    color: 'blue',
    size: 130
  });

  // State variables
  let interval = null;
  let linear = 0, angular = 0;

  // Joystick move event
  joystick.on('move', (_, data) => {
    if (data && data.vector) {
      // Cubic scaling for fine control
      linear = Math.pow(data.vector.y, 3) * 1 * velocityScale;
      angular = Math.pow(data.vector.x, 3) * velocityScale;
      status.textContent = `Linear: ${linear.toFixed(2)}, Angular: ${angular.toFixed(2)}`;
      console.log(`[DEBUG] linear: ${linear}, angular: ${angular}, scale: ${velocityScale}`);
    } else {
      linear = 0;
      angular = 0;
      status.textContent = "Idle";
      console.log('[DEBUG] No vector data from joystick.');
    }

    if (!interval) {
      interval = setInterval(() => {
        cmdVel.publish({
          linear: { x: linear, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: angular }
        });
        console.log('[DEBUG] Publishing:', { linear, angular });
      }, 100);
    }
  });

  // Joystick end event (released)
  joystick.on('end', () => {
    linear = 0;
    angular = 0;
    status.textContent = "Idle";
    cmdVel.publish({
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 }
    });
    if (interval) {
      clearInterval(interval);
      interval = null;
    }
    console.log('[DEBUG] Joystick released, sent zero velocities.');
  });

  // Initial status
  status.textContent = "Idle";
};

