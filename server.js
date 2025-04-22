// some sample server just to make sure that the server is running and im nt crazy
const WebSocket = require('ws');
const server = new WebSocket.Server({ host: '0.0.0.0', port: 8080 });

server.on('connection', (ws) => {
  console.log("🔥_🔥");
  ws.send("👋 Hello from server!");
});