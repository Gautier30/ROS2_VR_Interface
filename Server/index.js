const rclnodejs = require('rclnodejs')

const express = require('express')
const app = express()
const port = 3000

console.log('Running Server...');

// Create a ROS node and then print out the string message received from publishers
rclnodejs.init().then(() => {
  const node = rclnodejs.createNode('robot_pose_node');

  node.createSubscription('geometry_msgs/msg/PoseWithCovarianceStamped', '/amcl_pose', (msg) => {
    //console.log(`Received message: ${typeof msg}`, msg);
    const msgJSON = JSON.stringify(msg);
    //console.log(msgJSON)
    /*
    x_pos = msg['pose']['pose']['position']['x'];
    y_pos = msg['pose']['pose']['position']['y'];
    z_or = msg['pose']['pose']['orientation']['z'];
    w_or = msg['pose']['pose']['orientation']['w'];
    console.log(x_pos, y_pos, z_or, w_or)
    */
  });

  rclnodejs.spin(node);
});


app.get('/', (req, res) => {
  res.send('TEST')
})

app.listen(port, () => {
  console.log(`Example app listening on port ${port}`)
})

process.on('SIGINT', () => process.exit(0));
