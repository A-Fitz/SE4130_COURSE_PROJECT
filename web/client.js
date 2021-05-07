var socket = io();

socket.on('inputIn', function([x, y]) {
  document.getElementById("xTd").innerHTML = x;
  document.getElementById("yTd").innerHTML = y;
});

var emit_position = function() {
  var pos = [slider.position.x.toString(), slider.position.y.toString()];
  socket.emit("inputOut", pos);
};

// Set up slider input
$("#marker").draggable({
  containment: "#markerbounds",
  revertDuration: 200,
  revert: 'invalid',
  drag: function() {
    slider.get_position();
    emit_position();
  },
  stop: function() {
    slider.get_position();
    emit_position();
  },
});

// x_size, y_size, xmax, ymax, marker_size, round_to
slider.draw(250, 250, 1, 1, 50, 1);
slider.get_position();
emit_position();