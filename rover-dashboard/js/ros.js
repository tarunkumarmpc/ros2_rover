window.ros = null;
window.cameraTopic = null;

window.setupCameraFeed = function(rosInstance) {
  const feed = document.getElementById("camera-feed");
  const cameraSelect = document.getElementById("camera-select");
  const refreshBtn = document.getElementById("camera-refresh-btn");
  const placeholderText = "No image";

  // Map dropdown values to ROS topics
  const cameraTopics = {
    front: "/rover/front_camera/color/image_raw/compressed",
    back: "/rover/back_camera/color/image_raw/compressed"
  };

  // Helper to show placeholder
  const showPlaceholder = () => {
    feed.src = '';
    feed.alt = placeholderText;
    // Optionally, you could show an overlay or placeholder image here
    // For example, feed.src = 'placeholder.png';
  };

  // Subscription logic
  const subscribeToCameraTopic = (topicName) => {
    if (window.cameraTopic) {
      window.cameraTopic.unsubscribe();
    }
    window.cameraTopic = new ROSLIB.Topic({
      ros: rosInstance,
      name: topicName,
      messageType: "sensor_msgs/CompressedImage"
    });
    window.cameraTopic.subscribe(
      msg => {
        const mime = msg.format?.includes("png") ? "image/png" : "image/jpeg";
        feed.src = `data:${mime};base64,${msg.data}`;
        feed.alt = ""; // Clear placeholder alt when image is received
        window.logActivity(`Receiving images from ${topicName}`);
      },
      error => {
        window.logActivity(`Failed to subscribe to ${topicName}: ${error.message || 'Unknown error'}`);
        showPlaceholder();
      }
    );
    window.logActivity(`Subscribed to camera topic: ${topicName}`);
    showPlaceholder(); // Show placeholder until a new image arrives
  };

  // Initial subscription
  if (cameraSelect) {
    const initialTopic = cameraTopics[cameraSelect.value] || cameraTopics.front;
    subscribeToCameraTopic(initialTopic);
  } else {
    window.logActivity('Error: Camera select dropdown not found');
  }

  // Handle topic switching
  if (cameraSelect) {
    cameraSelect.addEventListener('change', () => {
      const selectedTopic = cameraTopics[cameraSelect.value];
      if (selectedTopic) {
        subscribeToCameraTopic(selectedTopic);
      } else {
        window.logActivity(`Error: Invalid camera selection: ${cameraSelect.value}`);
        showPlaceholder();
      }
    });
  }

  // Refresh button logic
  if (refreshBtn && feed) {
    refreshBtn.addEventListener('click', () => {
      showPlaceholder();
      if (window.logActivity) window.logActivity('Camera feed cleared on refresh');
    });
  }
};

window.logActivity = function(msg) {
  const log = document.getElementById('activity-log');
  const time = new Date().toLocaleTimeString();
  const entry = document.createElement('li');
  entry.textContent = `[${time}] ${msg}`;
  log.prepend(entry);
  while (log.children.length > 20) log.removeChild(log.lastChild);
};

window.setupRosoutLog = function(rosInstance) {
  const rosoutList = document.getElementById('rosout-log');
  const topic = new ROSLIB.Topic({
    ros: rosInstance,
    name: "/rosout",
    messageType: "rcl_interfaces/msg/Log"
  });
  topic.subscribe(msg => {
    const time = new Date(msg.stamp.sec * 1000).toLocaleTimeString();
    const levelMap = {
      10: "DEBUG",
      20: "INFO",
      30: "WARN",
      40: "ERROR",
      50: "FATAL"
    };
    const level = levelMap[msg.level] || "UNKNOWN";
    const entry = document.createElement('li');
    entry.innerHTML = `<span class="font-mono">${time}</span> <strong>${level}</strong>: ${msg.msg}`;
    rosoutList.prepend(entry);
    while (rosoutList.children.length > 20) rosoutList.removeChild(rosoutList.lastChild);
  });
};

window.setupClientList = function(rosInstance) {
  const summary = document.getElementById("client-summary");
  const list = document.getElementById("client-list");
  const topic = new ROSLIB.Topic({
    ros: rosInstance,
    name: "/connected_clients",
    messageType: "rosbridge_msgs/msg/ConnectedClients"
  });
  topic.subscribe(msg => {
    const clients = msg.clients || [];
    summary.textContent = `Connected Clients: ${clients.length}`;
    list.innerHTML = '';
    if (clients.length === 0) {
      list.innerHTML = '<div>No clients connected.</div>';
    } else {
      list.innerHTML = `
        <table>
          <thead><tr><th>IP</th><th>Connected</th></tr></thead>
          <tbody>
            ${clients.map(c => `<tr><td class="font-mono">${c.ip_address}</td><td>${new Date(c.connection_time.sec * 1000).toLocaleString()}</td></tr>`).join("")}
          </tbody>
        </table>`;
    }
  });
};

window.setupRoverModePublisher = function(rosInstance) {
  window.roverModePublisher = new ROSLIB.Topic({
    ros: rosInstance,
    name: "/rover_mode",
    messageType: "std_msgs/String"
  });
};

window.publishRoverMode = function(mode) {
  if (window.roverModePublisher) {
    window.roverModePublisher.publish({ data: mode });
    window.logActivity(`Mode switched to ${mode}`);
  }
};
