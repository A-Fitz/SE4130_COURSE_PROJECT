const express = require("express");
const http = require("http");
const net = require("net");
const { Server } = require("socket.io");

const app = express();
const server = http.createServer(app);
const io = new Server(server);
const tcpSocket = new net.Socket();

// Set up routes for resources
app.get("/js/socket.io.min.js", function (req, res) {
	res.sendFile(__dirname + "/node_modules/socket.io/client-dist/socket.io.min.js");
});
app.get("/js/jquery.min.js", function (req, res) {
	res.sendFile(__dirname + "/node_modules/jquery/dist/jquery.min.js");
});
app.get("/js/jquery-ui.min.js", function (req, res) {
	res.sendFile(__dirname + "/node_modules/jquery-ui-dist/jquery-ui.min.js");
});
app.get("/js/jquery.ui.touch-punch.min.js", function (req, res) {
	res.sendFile(__dirname + "/js/jquery.ui.touch-punch.min.js");
});
app.get("/", (req, res) => {
	res.sendFile(__dirname + "/index.html");
});
app.get("/js/client.js", (req, res) => {
	res.sendFile(__dirname + "/js/client.js");
});
app.get("/js/slider.js", (req, res) => {
	res.sendFile(__dirname + "/js/slider.js");
});
app.get("/style.css", (req, res) => {
	res.sendFile(__dirname + "/style.css");
});

const TCP_SERVER_IP = "192.168.10.1";
const TCP_SERVER_PORT = 8080;
const WEB_SERVER_PORT = 8080;
const SEND_PERIOD = 5; // ms

var x = 0,
	y = 0;
var canSend = true;

server.listen(WEB_SERVER_PORT, () => {
	console.log("Listening on *:" + WEB_SERVER_PORT);
});

io.on("connection", (webSocket) => {
	console.log("Web client connected.");

	tcpSocket.connect(TCP_SERVER_PORT, TCP_SERVER_IP, function () {
		tcpSocket.setEncoding("utf8");
		console.log("Connected to TCP server.");
		setInterval(sendPosition, SEND_PERIOD);
	});

	tcpSocket.on("data", function (recData) {
		io.emit("inputIn", recData.split(","));
		console.log("Received " + recData);
		canSend = true;
	});

	webSocket.on("inputOut", ([i_x, i_y]) => {
		x = i_x;
		y = i_y;
	});
});


// on timer
function sendPosition() {
	if (canSend) {
		tcpSocket.write(x + "," + y, "utf8", );
		console.log("Sent " + x + "," + y);
		canSend = false;
	}
}
