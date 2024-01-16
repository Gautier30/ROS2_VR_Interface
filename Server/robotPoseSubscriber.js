// THIS SCRIPTS CONNECTS TO THE WEB SOCKET SERVER, THEN OPENS A SUBSCRIBER TO ROBOT POSES ROS2 TOPICS

//* RCLNODEJS (ROS2)
const rclnodejs = require('rclnodejs')

//* WEBSOCKET
const WebSocket = require('ws')
// Create WebSocket connection.
const socket = new WebSocket("ws://localhost:3000");

// Connection opened
socket.addEventListener("open", (event) => {
    // Create a ROS node and then print out the string message received from publishers
    rclnodejs.init().then(() => {
        const node = rclnodejs.createNode('robot_pose_node');

        node.createSubscription('tf2_msgs/msg/TFMessage', '/tf', (msg) => {
            //console.log(msg)
            msg['identifier'] = 'robot_pose';
            const msgJSON = JSON.stringify(msg);
            socket.send(msgJSON);
        });

        rclnodejs.spin(node);
    });
  });

  process.on('SIGINT', () => process.exit(0));


