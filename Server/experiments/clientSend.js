const WebSocket = require('ws')
// Create WebSocket connection.
const socket = new WebSocket("ws://localhost:3000");

const dummyArray = new Uint8Array(1000);
const idString = "t"; //t for "test";
const strMsg = "This is a test message.";
// const combinedBuffer = Buffer.concat([
//   Buffer.from(idString, 'utf-8'),
//   dummyArray
// ]);

const combinedBuffer = Buffer.concat([
  Buffer.from(idString, 'utf-8'),
  Buffer.from(strMsg, 'utf-8')
]);

// Connection opened
socket.addEventListener("open", (event) => {

  socket.send(combinedBuffer, {binary:true});
  
});