// THIS SCRIPTS CONNECTS TO THE WEB SOCKET SERVER, THEN OPENS A SUBSCRIBER TO ROBOT CAMERA ROS2 TOPIC
const fs = require('fs');
const path = require('path');
const Jimp = require('jimp');

//* RCLNODEJS (ROS2)
const rclnodejs = require('rclnodejs');

//* WEBSOCKET
const WebSocket = require('ws')
// Create WebSocket connection.
const socket = new WebSocket("ws://localhost:3000");

let isSubscribed = false;

// Connection opened
socket.addEventListener("open", (event) => {
    // Create a ROS node and then print out the string message received from publishers
    rclnodejs.init().then(() => {
        const node = rclnodejs.createNode('robot_camera_node');

        socket.addEventListener("message", (event) => {
          const data = JSON.parse(event.data);

          if(data.identifier === 'camera_trigger'){
            if(data.command === 'start_camera_capture' && !isSubscribed){
              console.log('Starting camera capture...');
              camSubscriber = node.createSubscription('sensor_msgs/msg/Image', '/camera/image_raw', (message) => {
                console.log(message);
                message['identifier'] = 'raw_image';
                const msgJSON = JSON.stringify(message);
                socket.send(msgJSON);
                //saveImage(message);

              });
            isSubscribed = true;
            }

            else if(data.command === 'stop_camera_capture' && isSubscribed){
              console.log('Stopping camera capture...');
              node.destroySubscription(camSubscriber);
              isSubscribed = false;
            }
          }

        });

        rclnodejs.spin(node);

        process.on('SIGINT', () => {
          console.log('Exiting...');
          
          if(isSubscribed){
            node.destroySubscription(camSubscriber);
          }
          
          process.exit(0);
        });

        // Function to save the received image
        async function saveImage(imageMessage) {
          const imageData = Buffer.from(imageMessage.data);
          const width = imageMessage.width;
          const height = imageMessage.height;
          const imageName = 'frame.jpg'; // Use a consistent image name
          const imagePath = path.join('../deploy/textures', imageName); // Save images to the 'images' folder

          try {
            // Check if an image with the same name already exists
            if (fs.existsSync(imagePath)) {
              // Delete the existing image
              fs.unlinkSync(imagePath);
              console.log(`Deleted existing image: ${imagePath}`);
            }

            // Create a Jimp image from the received RGB8 data
            const image = await Jimp.read({
              data: imageData,
              width: width,
              height: height,
            });

            // Save the image as JPEG (or change format as needed)
            await image.writeAsync(imagePath);
            console.log(`Saved new image: ${imagePath}`);
          } catch (error) {
            console.error('Error saving image:', error);
          }
        }

    });
  });


