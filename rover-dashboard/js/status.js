window.setRosStatus = function(status) {
  const rosStatusIndicator = document.getElementById('ros-status-indicator');
  const rosStatusText = document.getElementById('ros-status-text');
  if (!rosStatusIndicator || !rosStatusText) return;
  if (status === 'connected') {
    rosStatusIndicator.style.background = '#22c55e';
    rosStatusText.textContent = 'ROS Connected';
    rosStatusText.style.color = '#22c55e';
  } else if (status === 'error') {
    rosStatusIndicator.style.background = '#ef4444';
    rosStatusText.textContent = 'ROS Error';
    rosStatusText.style.color = '#ef4444';
  } else if (status === 'closed') {
    rosStatusIndicator.style.background = '#f59e42';
    rosStatusText.textContent = 'ROS Disconnected';
    rosStatusText.style.color = '#f59e42';
  } else {
    rosStatusIndicator.style.background = '#f59e42';
    rosStatusText.textContent = 'Connecting...';
    rosStatusText.style.color = '#f59e42';
  }
};
setRosStatus();

