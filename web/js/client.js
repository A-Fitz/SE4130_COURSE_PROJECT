var socket = io();

socket.on('inputIn', function ([x, y]) {
	document.getElementById("xTd").innerHTML = x;
	document.getElementById("yTd").innerHTML = y;
});

var emit_position = function () {
	var pos = [slider.position.x.toString(), slider.position.y.toString()];
	socket.emit("inputOut", pos);
};

// Set up slider input
$("#marker").draggable({
	containment: '.map',
	revertDuration: 200,
	revert: 'invalid',
	drag: function(event, marker) {
		// Keep inside the circle
		var origin = slider.radius - slider.marker_radius;
		var x = marker.position.left - origin;
		var y = marker.position.top - origin;
		var l = Math.sqrt(x*x + y*y);
		var l_in = Math.min(slider.radius, l);
		marker.position.left = x/l*l_in + origin;
		marker.position.top = y/l*l_in + origin;
		
        slider.get_position();
		emit_position();
    },
	stop: function () {
		slider.get_position();
		emit_position();
	}
});

// containment_radius, xmax, ymax, marker_radius, round_to
slider.draw(240, 1, 1, 25, 1);
slider.get_position();
emit_position();
