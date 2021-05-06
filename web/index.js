const express = require('express');
const http = require('http');
const net = require('net');
const { Server } = require("socket.io");

const app = express();
const server = http.createServer(app);
const io = new Server(server);
const tcpSocket = new net.Socket();

// Set up routes for resources
app.get('/scripts/socket.io.min.js', function(req, res) {
    res.sendFile(__dirname + '/node_modules/socket.io/client-dist/socket.io.min.js');
});
app.get('/scripts/jquery.min.js', function(req, res) {
    res.sendFile(__dirname + '/node_modules/jquery/dist/jquery.min.js');
});
app.get('/scripts/jquery-ui.min.js', function(req, res) {
    res.sendFile(__dirname + '/node_modules/jquery-ui-dist/jquery-ui.min.js');
});
app.get('/', (req, res) => {
  res.sendFile(__dirname + '/index.html');
});
app.get('/client.js', (req, res) => {
  res.sendFile(__dirname + '/client.js');
});
app.get('/slider.js', (req, res) => {
  res.sendFile(__dirname + '/slider.js');
});

io.on('connection', (webSocket) => {
	console.log('Web client connected.');
	
	tcpSocket.connect(8080, '192.168.10.1', function() {
		console.log('Connected to TCP server.');
	});

	//tcpSocket.on('data', function(recData) {
	//	console.log(recData);
	//	//io.emit('inputIn', data);
	//});
	
	webSocket.on('inputOut', ([x,y]) => {
		tcpSocket.write('test');
	});
});

server.listen(80, () => {
  console.log('listening on *:80');
});