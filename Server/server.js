//* EXPRESS
const express = require('express')
const app = express()
const server = require('http').createServer(app);
const port = 3000

//* WEBSOCKET
const WebSocket = require('ws')
const wss = new WebSocket.Server({server:server});

console.log('Server is running.');

  wss.on('connection', function connection(ws) {
    ws.on('error', console.error);
    console.log('A new client is connected.')

    ws.on('message', function message(data, isBinary) { 
      wss.clients.forEach(function each(client){
        if (client !== ws && client.readyState === WebSocket.OPEN){
          client.send(data, { binary: isBinary });
        }

      });     
    });
  });

  
  app.get('/', (req, res) => {
    res.send('TEST')
  })
  
  server.listen(port, () => {
    console.log(`Example app listening on port ${port}`)
  })
  
  process.on('SIGINT', () => process.exit(0));