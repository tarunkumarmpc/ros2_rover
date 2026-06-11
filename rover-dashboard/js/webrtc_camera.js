// webrtc_camera.js

(() => {
  const video = document.getElementById('camera-feed');
  if (!video) {
    console.error('No <video> element with id "camera-feed" found.');
    return;
  }

  let peerConnection = null;
  let signalingSocket = null;

  // Replace these with your ROS WebRTC signaling server address and port
  const SIGNALING_SERVER_URL = 'ws://<ROS_SERVER_IP>:<SIGNALING_PORT>/signaling';

  // Initialize WebSocket connection to signaling server
  function connectSignaling() {
    signalingSocket = new WebSocket(SIGNALING_SERVER_URL);

    signalingSocket.onopen = () => {
      console.log('Signaling socket connected');
      // Request an offer for the camera topic (adjust topic name as needed)
      signalingSocket.send(JSON.stringify({
        type: 'request_offer',
        topic: '/cam0/image_raw'
      }));
    };

    signalingSocket.onmessage = async (event) => {
      const msg = JSON.parse(event.data);

      switch (msg.type) {
        case 'offer':
          await handleOffer(msg.offer);
          break;
        case 'candidate':
          await handleCandidate(msg.candidate);
          break;
        case 'answer':
          // Usually client does not receive answer, but handle if needed
          break;
        default:
          console.warn('Unknown signaling message type:', msg.type);
      }
    };

    signalingSocket.onerror = (error) => {
      console.error('Signaling socket error:', error);
    };

    signalingSocket.onclose = () => {
      console.warn('Signaling socket closed, retrying in 3 seconds...');
      setTimeout(connectSignaling, 3000);
    };
  }

  async function handleOffer(offer) {
    if (peerConnection) {
      peerConnection.close();
      peerConnection = null;
    }

    peerConnection = new RTCPeerConnection();

    // Send ICE candidates to signaling server
    peerConnection.onicecandidate = (event) => {
      if (event.candidate) {
        signalingSocket.send(JSON.stringify({
          type: 'candidate',
          candidate: event.candidate
        }));
      }
    };

    // When remote stream arrives, set it as video source
    peerConnection.ontrack = (event) => {
      if (video.srcObject !== event.streams[0]) {
        console.log('Received remote stream');
        video.srcObject = event.streams[0];
      }
    };

    await peerConnection.setRemoteDescription(new RTCSessionDescription(offer));
    const answer = await peerConnection.createAnswer();
    await peerConnection.setLocalDescription(answer);

    signalingSocket.send(JSON.stringify({
      type: 'answer',
      answer: peerConnection.localDescription
    }));
  }

  async function handleCandidate(candidate) {
    if (!peerConnection) {
      console.warn('Received ICE candidate but peerConnection is null');
      return;
    }
    try {
      await peerConnection.addIceCandidate(new RTCIceCandidate(candidate));
    } catch (e) {
      console.error('Error adding received ICE candidate', e);
    }
  }

  // Start connection
  connectSignaling();
})();

