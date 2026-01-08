// ROS connection
const ros = new ROSLIB.Ros({
    url: 'ws://127.0.0.1:9090'
});

ros.on('connection', () => {
    document.getElementById('status').innerHTML = 'Connected';
    console.log('Connected to rosbridge');
});

ros.on('error', (error) => {
    document.getElementById('status').innerHTML = 'Error: ' + error;
    console.log('Error connecting to rosbridge', error);
});

ros.on('close', () => {
    document.getElementById('status').innerHTML = 'Disconnected';
    console.log('Connection to rosbridge closed');
});

// Topics
const cmdVelTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/turtle1/cmd_vel',
    messageType: 'geometry_msgs/msg/Twist'
});

const poseTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/turtle1/pose',
    messageType: 'turtlesim/msg/Pose'
});

// Teleport service
const teleportClient = new ROSLIB.Service({
    ros: ros,
    name: '/teleport_turtle',
    serviceType: 'std_srvs/srv/SetBool'
});


// Canvas
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
let turtleX = 150, turtleY = 150;

// Subscribe to pose
poseTopic.subscribe((message) => {
    document.getElementById('pose_x').innerText = message.x.toFixed(2);
    document.getElementById('pose_y').innerText = message.y.toFixed(2);
    document.getElementById('pose_theta').innerText = message.theta.toFixed(2);
    
    turtleX = (message.x / 11) * 300;
    turtleY = 300 - (message.y / 11) * 300;
    draw();
});

// Teleop
function sendCmd(linearX, angularZ) {
    const twist = new ROSLIB.Message({
        linear: { x: linearX, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: angularZ }
    });
    console.log('cmd_vel:', {linearX, angularZ});
    cmdVelTopic.publish(twist);
}

// Teleport 
function teleportCorner(cornerId) {
    console.log(`Teleport corner ${cornerId} (sending true)`);
    const request = new ROSLIB.ServiceRequest({ data: true });
    teleportClient.callService(request, (result) => {
        console.log('Service result:', result);
    });
}


// Draw
function draw() {
    ctx.fillStyle = '#f0f0f0';
    ctx.fillRect(0, 0, 300, 300);
    ctx.strokeStyle = '#000';
    ctx.lineWidth = 2;
    ctx.strokeRect(10, 10, 280, 280);
    
    ctx.save();
    ctx.translate(turtleX, turtleY);
    ctx.rotate(-parseFloat(document.getElementById('pose_theta').innerText || 0));
    ctx.fillStyle = '#00f';
    ctx.beginPath();
    ctx.moveTo(0, -15);
    ctx.lineTo(12, 10);
    ctx.lineTo(-12, 10);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
}

// Keyboard
document.addEventListener('keydown', (e) => {
    switch(e.code) {
        case 'ArrowUp': sendCmd(1.0, 0.0); break;
        case 'ArrowDown': sendCmd(-0.5, 0.0); break;
        case 'ArrowLeft': sendCmd(0.0, 0.5); break;
        case 'ArrowRight': sendCmd(0.0, -0.5); break;
    }
});
document.addEventListener('keyup', () => sendCmd(0.0, 0.0));
