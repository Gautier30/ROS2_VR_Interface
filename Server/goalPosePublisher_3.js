const rclnodejs = require('rclnodejs');
const WebSocket = require('ws');

rclnodejs.init().then(() => {
    const node = rclnodejs.createNode('robot_goal_node', 'waffle3');
    const publisher = node.createPublisher('geometry_msgs/msg/PoseStamped', '/waffle3/goal_pose');
    
    // WebSocket connection
    const socket = new WebSocket("ws://localhost:3000");

    // Connection opened
    socket.addEventListener("message", (event) => {
        if(!(event.data instanceof Blob) && typeof event.data != "object"){
            var msgParsed = JSON.parse(event.data);
            if (msgParsed['identifier'] === 'waffle3/goal_pose') {
                console.log('Going to', msgParsed['0'], msgParsed['2'])
                // Update and publish the message on ROS topic
                publisher.publish({
                    header: {
                        stamp: {
                            sec: 0,
                            nanosec: 0
                        },
                        frame_id: 'map',
                    },
                    pose: {
                        position: {
                            x: msgParsed['0'],
                            y: -msgParsed['2'], // Negating the Z-axis coordinate
                            z: 0.0
                        },
                        orientation: {
                            x: 0.0,
                            y: 0.0,
                            z: 0.0,
                            w: 0.0
                        }
                    }
                });
            }
        }
    });

    rclnodejs.spin(node);

    process.on('SIGINT', () => {
        rclnodejs.shutdown();
        process.exit(0);
    });
}).catch((e) => {
    console.log(e);
});