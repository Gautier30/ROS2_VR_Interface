const rclnodejs = require('rclnodejs');
const WebSocket = require('ws');

rclnodejs.init().then(() => {
    const node = rclnodejs.createNode('cmdVel_VR_node');
    const publisher = node.createPublisher('geometry_msgs/msg/Twist', '/cmd_vel');
    
    // WebSocket connection
    const socket = new WebSocket("ws://localhost:3000");

    var newLinVel = 0.0;
    var currentLinVel = 0.0;

    var newAngVel = 0.0;
    var currentAngVel = 0.0;


    // Connection opened
    socket.addEventListener("message", (event) => {
        var msgParsed = JSON.parse(event.data);
        
        if (msgParsed['identifier'] === 'lin_vel') {
            newLinVel = msgParsed['vel'];
            if(newLinVel != currentLinVel){
                currentLinVel = newLinVel;
            }
        }

        if (msgParsed['identifier'] === 'ang_vel') {
            newAngVel = msgParsed['vel'];
            if(newAngVel != currentAngVel){
                currentAngVel = newAngVel;
            }
        }
            
        // Update and publish the message on ROS topic
        publisher.publish({
            
            linear: {
                x: currentLinVel,
                y: 0.0,
                z: 0.0
            },
            angular: {
                x: 0.0,
                y: 0.0,
                z: currentAngVel
            }
            
        });
        

    });

    rclnodejs.spin(node);

    process.on('SIGINT', () => {
        rclnodejs.shutdown();
        process.exit(0);
    });
}).catch((e) => {
    console.log(e);
});