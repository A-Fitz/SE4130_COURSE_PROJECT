// https://codepen.io/tyler-murphy/pen/tHsAu
var slider = {

  get_position: function() {
    var marker_pos = $('#marker').position();
    var left_pos = marker_pos.left + slider.marker_size / 2;
    var top_pos = marker_pos.top + slider.marker_size / 2;

    slider.position = {
      left: left_pos,
      top: top_pos,
      x: Math.round(slider.round_factor.x * (left_pos * slider.xmax / slider.width)) / slider.round_factor.x,
      y: Math.round((slider.round_factor.y * (slider.height - top_pos) * slider.ymax / slider.height)) / slider.round_factor.y,
    };
  },

  draw: function(x_size, y_size, xmax, ymax, marker_size, round_to) {

    slider.marker_size = marker_size;
    slider.height = y_size;
    slider.width = x_size;
    slider.xmax = xmax;
    slider.ymax = ymax;
    round_to = Math.pow(10, round_to);
    slider.round_factor = {
      x: round_to,
      y: round_to,
    };

    $("#markerbounds").css({
      "width": (x_size + marker_size).toString() + 'px',
      "height": (y_size + marker_size).toString() + 'px',
    });
    $("#box").css({
      "width": x_size.toString() + 'px',
      "height": y_size.toString() + 'px',
      "top": marker_size / 2,
      "left": marker_size / 2,
    });
    $("#marker").css({
      "width": marker_size.toString() + 'px',
      "height": marker_size.toString() + 'px',
      "inset": (y_size - marker_size).toString() / 2 + 'px ' + 'auto auto ' + (x_size - marker_size).toString() / 2 + 'px'
    });
    $("#widget").css({
      "width": (x_size + marker_size).toString() + 'px',
    });

  },
};