// server.js
const WebSocket = require('ws');
const server = new WebSocket.Server({ host: '0.0.0.0', port: 8023 });

server.on('connection', (ws) => {
  console.log("🔥_🔥");
  ws.send("👋 Hello from server!");
});