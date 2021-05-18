// https://codepen.io/tyler-murphy/pen/tHsAu
var slider = {

	get_position: function () {
		var marker_pos = $('#marker').position();
		slider.position = {
			x: Math.round((slider.round_factor.x * (marker_pos.left + slider.marker_radius - slider.radius) * slider.xmax / slider.radius)) / slider.round_factor.x,
			y: Math.round((slider.round_factor.y * (slider.radius - slider.marker_radius - marker_pos.top) * slider.ymax / slider.radius)) / slider.round_factor.y,
		};
	},

	draw: function (containment_radius, xmax, ymax, marker_radius, round_to) {
		slider.marker_radius = marker_radius;
		slider.radius = containment_radius;
		slider.xmax = xmax;
		slider.ymax = ymax;
		round_to = Math.pow(10, round_to);
		slider.round_factor = {
			x: round_to,
			y: round_to,
		};

		$("#markerbounds").css({
			"width": ((containment_radius * 2)).toString() + 'px',
			"height": ((containment_radius * 2)).toString() + 'px',
		});
		$("#containment").css({
			"width": (containment_radius * 2).toString() + 'px',
			"height": (containment_radius * 2).toString() + 'px',
		});
		$("#marker").css({
			"width": (marker_radius * 2).toString() + 'px',
			"height": (marker_radius * 2).toString() + 'px',
			"inset": ((containment_radius * 2) - (marker_radius * 2)).toString() / 2 + 'px ' + 'auto auto ' + ((containment_radius * 2) - (marker_radius * 2)).toString() / 2 + 'px'
		});
		$("#widget").css({
			"width": ((containment_radius * 2) + (marker_radius * 2)).toString() + 'px',
		});
	},
};
