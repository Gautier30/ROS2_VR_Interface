//* RCLNODEJS (ROS2)
const rclnodejs = require('rclnodejs')

//* WEBSOCKET
const WebSocket = require('ws')
// Create WebSocket connection.
const socket = new WebSocket("ws://localhost:3000");

startTime = new Date();

// Connection opened
socket.addEventListener("open", (event) => {

    // Create a ROS node and then print out the string message received from publishers
    rclnodejs.init().then(() => {
        const node = rclnodejs.createNode('robot_camera_node');

        node.createSubscription('sensor_msgs/msg/Image', '/camera/image_raw', (msg) => {
            let buffer = new ArrayBuffer(msg.data.length)
            buffer = msg.data
            socket.send(buffer);
            endTime = new Date();
            var timeDiff = endTime - startTime;
            var seconds = timeDiff/1000;
            var frequency = 1/seconds;
            //console.log(frequency);
            startTime = endTime;
            //console.log(msg);
        });

        rclnodejs.spin(node);
    });

});

process.on('SIGINT', () => process.exit(0));  