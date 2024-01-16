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

        const node = rclnodejs.createNode('robot_camera_node', 'waffle3');

        socket.addEventListener("message", (event) => {
            if(!(event.data instanceof Blob) && typeof event.data != "object"){
                const receivedMessage = JSON.parse(event.data);

                if(receivedMessage.identifier === "waffle3/camera_trigger"){

                    if(receivedMessage.command === "capture_ON"){
                        console.log("Capture..");
                        

                        camSubscriber = node.createSubscription('sensor_msgs/msg/Image', '/waffle3/camera/image_raw', (msg) => {
                            let buffer = new ArrayBuffer(msg.data.length)
                            buffer = msg.data
                            socket.send(buffer);
                            node.destroySubscription(camSubscriber);                            
                
                        });

                    }

                }
            }

            // else{
            //     node.destroySubscription(camSubscriber);
            // }
            
        });
        rclnodejs.spin(node);
    });
  });

  process.on('SIGINT', () => process.exit(0));
