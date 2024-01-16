const WebSocket = require('ws')
// Create WebSocket connection.
const socket = new WebSocket("ws://localhost:3000");

// Connection opened
socket.addEventListener("open", (event) => {
});

// Listen for messages
socket.addEventListener("message", (event) => {
  // console.log(event.data);

  if (event.data instanceof Buffer) {
    // Assuming you receive binary data
    const data = new Uint8Array(event.data);

    // Parse the identifier (assuming it's the first byte)
    const identifier = String.fromCharCode(data[0]);
    // console.log(identifier);
    
    // Extract the binary data (excluding the identifier)
    const binaryData = data.slice(1);

    let strMsg = "";

    if (identifier === 't') {

      for(i=0; i < binaryData.length; i++){
        strMsg += String.fromCharCode(binaryData[i]);
      }

      console.log("Binary message:");
      console.log(binaryData);
      console.log("Decoded to string:")
      console.log(strMsg);
    }
  }

  else{
    console.log(typeof(event.data));
    console.log(event.data)
  }
});