//* RCLNODEJS (ROS2)
const rclnodejs = require('rclnodejs')
startTime = new Date();

// Create a ROS node and then print out the string message received from publishers
rclnodejs.init().then(() => {
    const node = rclnodejs.createNode('robot_camera_node');

    node.createSubscription('sensor_msgs/msg/Image', '/camera/image_raw', (msg) => {
        endTime = new Date();
        var timeDiff = endTime - startTime;
        var seconds = timeDiff/1000;
        var frequency = 1/seconds;
        console.log(frequency);
        startTime = endTime;
        //console.log(msg);
    });

    rclnodejs.spin(node);
});

  process.on('SIGINT', () => process.exit(0));