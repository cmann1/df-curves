#include '../lib/std.cpp';
#include '../lib/debug/Debug.cpp';
#include '../lib/drawing/common.cpp';
#include '../lib/input/Mouse.cpp';
#include '../lib/enums/GVB.cpp';
#include '../lib/enums/VK.cpp';

#include 'CurveTypeUtils.cpp';
#include 'MultiCurve.cpp';
#include 'MultiCurveDebug.cpp';

class script : MultiCurveDebugColourCallback
{
	
	[persist] float speed = 1;
	[persist] bool low_precision = false;
	[persist] bool arc_length_interpolation = true;
	[persist] bool adaptive_stretch_factor = true;
	[persist] bool adjust_initial_binary_factor = true;
	[persist] bool render_arc_lengths = false;
	[persist] bool render_segment_bboxes = true;
	[persist] float max_mouse_distance = 0;
	
	scene@ g;
	input_api@ input;
	editor_api@ editor;
	camera@ cam;
	canvas@ c;
	Mouse mouse(false);
	float zoom;
	float zoom_factor = 1;
	bool mouse_in_scene;
	bool space_down;
	bool ctrl_down;
	bool shift_down;
	bool alt_down;
	bool is_polling_keyboard;
	
	Debug debug;
	
	EditState state = Idle;
	
	CurveControlPoint@ hover_point;
	CurveVertex@ hover_vertex;
	bool hover_is_vertex;
	CurveControlPoint@ drag_point;
	CurveVertex@ drag_vertex;
	bool drag_is_vertex;
	int hover_vertex_index;
	int hover_segment_index;
	int hover_control_point_index;
	int drag_vertex_index;
	int drag_segment_index;
	int drag_control_point_index;
	float drag_ox, drag_oy;
	float drag_cp_x, drag_cp_y;
	ClosestPointTest closest_point;
	
	MultiCurve curve;
	MultiCurveDebug debug_draw;
	CurveChange curve_changed = None;
	
	uint seed = 0;
	bool is_rand;
	
	float t = 0;
	
	textfield@ display_txt;
	float display_txt_timer = -1;
	float display_txt_x, display_txt_y;
	
	float segment_alpha = 1;
	
	script()
	{
		@g = get_scene();
		@input = get_input_api();
		@editor = get_editor_api();
		@c = create_canvas(false, 22, 22);
		mouse.use_input(input);
		
		@display_txt = create_textfield();
		display_txt.align_horizontal(0);
		display_txt.align_vertical(1);
		display_txt.set_font('envy_bold', 20);
		
		@cam = get_active_camera();
		zoom = cam.editor_zoom();
		zoom_factor = 1 / zoom;
		
		debug_draw.curve_segments = 6;
		debug_draw.adaptive_angle = 2;
		debug_draw.adaptive_max_subdivisions = 5;
		@debug_draw.segment_colour_callback = this;
	}
	
	void on_editor_start()
	{
		update_curve_precision();
		
		curve.closed = true;
		curve.type = CubicBezier;
		
		recreate_spline();
		
		curve.invalidate();
		curve.validate();
		curve_changed = None;
	}
	
	void editor_var_changed(var_info@ info)
	{
		const string name = info.name;
		
		if(name == 'low_precision' || name == 'adaptive_stretch_factor')
		{
			update_curve_precision();
			curve_changed = All;
		}
	}
	
	void editor_step()
	{
		if(display_txt_timer > 0)
		{
			display_txt_timer = max(display_txt_timer - 1, 0.0);
		}
		else if(display_txt_timer > -1)
		{
			display_txt_timer = max(display_txt_timer - DT * 16, -1.0);
		}
		
		zoom = cam.editor_zoom();
		zoom_factor = 1 / zoom;
		
		debug_draw.adaptive_min_length = 8 * zoom_factor;
		debug_draw.adaptive_angle = map_clamped(zoom_factor, 0.1, 4, 2, 25);
		debug_draw.segment_bounding_box_width = render_segment_bboxes ? 2 : 0;
		
		mouse_in_scene = !editor.mouse_in_gui() && editor.editor_tab() == 'Scripts';
		const bool block_mouse = editor.mouse_in_gui() || space_down;
		mouse.step(block_mouse);
		
		space_down = input.key_check_gvb(GVB::Space);
		ctrl_down = input.key_check_gvb(GVB::Control);
		shift_down = input.key_check_gvb(GVB::Shift);
		alt_down = input.key_check_gvb(GVB::Alt);
		is_polling_keyboard = input.is_polling_keyboard();
		
		if(state == Idle)
		{
			closest_point.found = curve.closest_point(
				mouse.x, mouse.y, closest_point.i, closest_point.t, closest_point.x, closest_point.y,
				max_mouse_distance * zoom_factor, 1, arc_length_interpolation, adjust_initial_binary_factor, true);
			
			if(closest_point.found)
			{
				closest_point.from_x = mouse.x;
				closest_point.from_y = mouse.y;
				closest_point.dx = closest_point.from_x - closest_point.x;
				closest_point.dy = closest_point.from_y - closest_point.y;
				closest_point.dist = sqrt(closest_point.dx * closest_point.dx + closest_point.dy * closest_point.dy);
				closest_point.nx = closest_point.dist != 0 ? closest_point.dx / closest_point.dist : 0;
				closest_point.ny = closest_point.dist != 0 ? closest_point.dy / closest_point.dist : 0;
				
				closest_point.hover = closest_point.dist <= debug_draw.line_width * 1.5 * zoom_factor;
			}
			else
			{
				closest_point.hover = false;
			}
		}
		else
		{
			closest_point.found = false;
		}
		
		switch(state)
		{
			case Idle: state_idle(); break;
			case DragVertex: state_drag_vertex(); break;
			case DragWeight: state_drag_weight(); break;
		}
		
		if(check_pressed(VK::V))
		{
			recreate_spline();
		}
		if(check_pressed(VK::M))
		{
			curve.closed = !curve.closed;
			curve_changed = All;
		}
		if(check_pressed(VK::K) && curve.type == BSpline)
		{
			curve.b_spline_clamped = !curve.b_spline_clamped;
			curve_changed = All;
		}
		if(check_pressed(VK::N))
		{
			is_rand = !is_rand;
			recreate_spline();
		}
		if(check_pressed(VK::LeftBrace) || check_pressed(VK::RightBrace))
		{
			curve.type = CurveType(mod(curve.type + (check_pressed(VK::LeftBrace) ? -1 : 1), BSpline + 1));
			curve.invalidate();
			curve.validate();
			display_text_at_curve('Curve type: ' + Curve::get_type_name(curve.type), 30);
		}
		if(check_pressed(VK::L) && curve.type == CatmullRom)
		{
			switch(curve.end_controls)
			{
				case CurveEndControl::AutomaticAngle: curve.end_controls = CurveEndControl::Automatic; break;
				case CurveEndControl::Automatic: curve.end_controls = CurveEndControl::Manual; break;
				case CurveEndControl::Manual: curve.end_controls = CurveEndControl::AutomaticAngle; break;
			}
			curve_changed = All;
		}
		if(check_pressed(VK::P))
		{
			low_precision = !low_precision;
			update_curve_precision();
			curve_changed = All;
		}
		if(check_pressed(VK::U))
		{
			adaptive_stretch_factor = !adaptive_stretch_factor;
			update_curve_precision();
			curve_changed = All;
			editor_sync_vars_menu();
		}
		if(check_pressed(VK::I))
		{
			arc_length_interpolation = !arc_length_interpolation;
			editor_sync_vars_menu();
		}
		if(check_pressed(VK::Y))
		{
			adjust_initial_binary_factor = !adjust_initial_binary_factor;
			editor_sync_vars_menu();
		}
		if(check_pressed(VK::OemComma))
		{
			render_arc_lengths = !render_arc_lengths;
			editor_sync_vars_menu();
		}
		
		if(mouse_in_scene && mouse.scroll != 0)
		{
			switch(curve.type)
			{
				case CatmullRom:
					curve.tension = clamp(curve.tension - mouse.scroll * 0.1, 0.25, 30.0);
					display_text_at_curve('Tension: ' + str(curve.tension), 25);
					curve_changed = All;
					break;
				case BSpline:
					curve.b_spline_degree = clamp(curve.b_spline_degree - mouse.scroll, 2, 7);
					display_text_at_curve('B-Spline degree: ' + curve.b_spline_degree, 25);
					if(curve.is_invalidated)
					{
						curve_changed = All;
					}
					break;
			}
		}
		
		if(mouse_in_scene && mouse.middle_press)
		{
			curve.tension = 1;
			curve_changed = All;
		}
		
		if(curve_changed != None)
		{
			if(curve_changed == All)
			{
				curve.invalidate();
			}
			
			curve.validate();
			curve_changed = None;
		}
		
		t += speed * 0.25 * DT;
		
		debug.step();
	}
	
	void editor_draw(float sub_frame)
	{
		debug_draw.clip = true;
		cam.get_layer_draw_rect(sub_frame, c.layer(), debug_draw.clip_x1, debug_draw.clip_y1, debug_draw.clip_x2, debug_draw.clip_y2);
		debug_draw.clip_x2 += debug_draw.clip_x1;
		debug_draw.clip_y2 += debug_draw.clip_y1;
		
		//debug_draw.clip_x1 += 250 * zoom_factor;
		//debug_draw.clip_y1 += 250 * zoom_factor;
		//debug_draw.clip_x2 -= 250 * zoom_factor;
		//debug_draw.clip_y2 -= 250 * zoom_factor;
		//outline_rect_outside(g, 22, 22, debug_draw.clip_x1, debug_draw.clip_y1, debug_draw.clip_x2, debug_draw.clip_y2, 5*zoom_factor, 0x88ff0000);
		
		if(closest_point.found && @hover_point == null)
		{
			const float nx =  closest_point.ny * 30 * zoom_factor;
			const float ny = -closest_point.nx * 30 * zoom_factor;
			g.draw_line_world(22, 22, closest_point.from_x, closest_point.from_y, closest_point.x, closest_point.y, 2 * zoom_factor, 0x33ffffff);
			draw_dot(
				g, 22, closest_point.hover ? 24 : 22, closest_point.x, closest_point.y, 3 * zoom_factor,
				closest_point.hover ? 0xffffffff : 0x55ffffff, 45);
			g.draw_line_world(22, 22, closest_point.x - nx, closest_point.y - ny, closest_point.x + nx, closest_point.y + ny, 1 * zoom_factor, 0x33ffffff);
		}
		
		if(render_arc_lengths)
		{
			segment_alpha = 0.25;
			debug_draw.normal_width = 0;
			debug_draw.draw_curve(c, curve, zoom_factor);
			segment_alpha = 1;
			debug_draw.normal_width = 1;
			
			debug_draw.draw_outline(c, curve, zoom_factor);
			debug_draw.draw_control_points(c, curve, zoom_factor);
			debug_draw.draw_arch_lengths(c, curve, zoom_factor);
			debug_draw.draw_vertices(c, curve, zoom_factor);
			debug_draw.draw_segment_labels(c, curve, zoom_factor);
			debug_draw.draw_hovered(c, curve, zoom_factor);
		}
		else
		{
			debug_draw.draw(c, curve, zoom_factor);
		}
		
		//float x, y, nx, ny;
		//curve.eval(-1, abs(t % 2 - 1), x, y, nx, ny);
		//curve.eval(t % 1, x, y, nx, ny);
		//draw_dot(g, 22, 22, x, y, 4 * zoom_factor, 0xffffffff, 45);
		
		if(display_txt_timer > -1)
		{
			display_txt.colour(display_txt_timer > 0 ? 0xffffffff : multiply_alpha(0xffffffff, 1 + display_txt_timer));
			display_txt.draw_world(22, 22, display_txt_x, display_txt_y, zoom_factor, zoom_factor, 0);
		}
		
		debug.draw(sub_frame);
	}
	
	void state_idle()
	{
		// Update hover.
		if(
			mouse_in_scene && (
				get_control_point_at_mouse(hover_point, hover_segment_index, hover_vertex_index, hover_control_point_index) ||
				get_vertex_at_mouse(hover_point, hover_segment_index, hover_vertex_index, hover_control_point_index))
		)
		{
			@hover_vertex = cast<CurveVertex@>(hover_point);
			hover_is_vertex = @hover_vertex != null;
			
			debug_draw.hovered_vertex_index = hover_vertex_index;
			debug_draw.hovered_control_point_index = hover_control_point_index;
		}
		else
		{
			@hover_point = null;
			@hover_vertex = null;
			hover_segment_index = -1;
			hover_vertex_index = -1;
			hover_control_point_index = 0;
			drag_vertex_index = -1;
			drag_control_point_index = 0;
			debug_draw.hovered_vertex_index = -1;
			debug_draw.hovered_control_point_index = 0;
		}
		
		if(!mouse_in_scene)
			return;
		
		// Change vertex type.
		if(mouse.left_press && shift_down && @hover_point != null)
		{
			CurveControlType new_type = Smooth;
			switch(hover_point.type)
			{
				case Square: new_type = Smooth; break;
				case Smooth: new_type = Square; break;
			}
			
			curve.set_control_type(hover_point, new_type);
			curve_changed = Validate;
			
			display_text(Curve::get_control_type_name(hover_point.type), 30);
			return;
		}
		
		// Reset weight.
		if(mouse.right_double_click && alt_down)
		{
			if(@hover_point != null)
			{
				if(curve.type != CatmullRom)
				{
					hover_point.weight = 1;
					curve.invalidate(hover_vertex_index, !hover_is_vertex);
					curve_changed = Validate;
				}
				else if(hover_is_vertex)
				{
					hover_vertex.tension = 1;
					curve.invalidate(hover_vertex_index, true);
					curve_changed = Validate;
				}
			}
			return;
		}
		
		// Start dragging.
		if(mouse.left_press && @hover_point != null)
		{
			start_drag_hover();
			init_drag();
			
			state = DragVertex;
			return;
		}
		
		// Start dragging weight.
		if(mouse.right_press && alt_down && @hover_point != null)
		{
			start_drag_hover();
			drag_ox = mouse.x;
			drag_oy = drag_point.weight;
			state = DragWeight;
			return;
		}
		
		// Insert vertex.
		if(mouse.left_press && ctrl_down && @hover_point == null)
		{
			int i = curve.get_adjusted_segment_index(closest_point.i, closest_point.t);
			int new_index;
			
			if(closest_point.hover)
			{
				new_index = curve.insert_vertex(i, closest_point.t);
			}
			else if(closest_point.found)
			{
				new_index = curve.insert_vertex(i, mouse.x, mouse.y);
				curve.init_bezier_control_points(true, new_index - 1, 2);
			}
			else
			{
				new_index = curve.insert_vertex(curve.vertex_count, mouse.x, mouse.y);
				curve.init_bezier_control_points(true, new_index);
			}
			
			if(new_index != -1)
			{
				curve_changed = Validate;
				
				CurveVertex@ v = curve.vertices[new_index];
				
				debug_draw.hovered_vertex_index = new_index;
				debug_draw.hovered_control_point_index = 0;
				@drag_point = v;
				@drag_vertex = v;
				drag_is_vertex = true;
				drag_segment_index = new_index;
				drag_vertex_index = new_index;
				drag_control_point_index = 0;
				
				init_drag();
				state = DragVertex;
			}
			return;
		}
	}
	
	void state_drag_vertex()
	{
		if(!mouse.left_down)
		{
			stop_drag();
			state = Idle;
			return;
		}
		
		if(mouse.moved)
		{
			drag_point.x = mouse.x + drag_ox;
			drag_point.y = mouse.y + drag_oy;
			
			if(drag_is_vertex && curve.type == QuadraticBezier && drag_vertex.quad_control_point.type != Square)
			{
				drag_vertex.quad_control_point.x = drag_cp_x - drag_vertex.x;
				drag_vertex.quad_control_point.y = drag_cp_y - drag_vertex.y;
			}
			
			curve.invalidate(drag_segment_index, !drag_is_vertex);
			curve_changed = Validate;
		}
	}
	
	void state_drag_weight()
	{
		if(!mouse.right_down)
		{
			stop_drag();
			state = Idle;
			return;
		}
		
		if(mouse.moved)
		{
			if(curve.type == CatmullRom)
			{
				if(drag_is_vertex)
				{
					drag_vertex.tension = clamp(drag_oy + (mouse.x - drag_ox) / zoom_factor * 0.2, 0.25, 30.0);
					display_txt.text('Tension: ' + str(drag_vertex.tension));
				}
				
				curve.invalidate(drag_segment_index, true);
				curve_changed = Validate;
			}
			else
			{
				drag_point.weight = clamp(drag_oy + (mouse.x - drag_ox) / zoom_factor * 0.2, drag_is_vertex ? 0.05 : 0.001, 60.0);
				display_txt.text('Weight: ' + str(drag_point.weight));
				
				curve.invalidate(drag_segment_index, !drag_is_vertex);
				curve_changed = Validate;
			}
		}
		
		if(curve.type == CatmullRom)
		{
			if(drag_is_vertex)
			{
				display_txt.text('Tension: ' + str(drag_vertex.tension));
			}
		}
		else
		{
			display_txt.text('Weight: ' + str(drag_point.weight));
		}
		
		display_txt_timer = 1;
		display_txt_x = mouse.x;
		display_txt_y = mouse.y - 5 * zoom_factor;
	}
	
	bool get_vertex_at_mouse(CurveControlPoint@ &out result, int &out segment_index, int &out vertex_index, int &out control_point_index, const float size=5)
	{
		const float max_dist = size * zoom_factor;
		@result = null;
		float closest_dist = MAX_FLOAT;
		segment_index = -1;
		vertex_index = -1;
		control_point_index = 0;
		
		const int start_i = curve.end_controls == CurveEndControl::Manual && curve.type == CatmullRom
			? -2 : 0;
		
		for(int i = start_i; i < curve.vertex_count; i++)
		{
			CurveControlPoint@ p = i >= 0
				? curve.vertices[i]
				: i == -1 ? curve.control_point_start : curve.control_point_end;
			const float dist = distance(p.x, p.y, mouse.x, mouse.y);
			
			if(dist <= max_dist && dist < closest_dist)
			{
				closest_dist = dist;
				@result = p;
				segment_index = i == -2 ? curve.vertex_count - 2 : clamp(i, 0, curve.vertex_count - 1);
				vertex_index = i == -2 ? -1 : i == -1 ? curve.vertex_count : i;
				control_point_index = i == -2 ? -1 : i == -1 ? 1 : 0;
			}
		}
		
		return @result != null;
	}
	
	bool get_control_point_at_mouse(CurveControlPoint@ &out result, int &out segment_index, int &out vertex_index, int &out control_point_index, const float size=4)
	{
		if(curve.type != QuadraticBezier && curve.type != CubicBezier)
			return false;
		
		const float max_dist = size * size * zoom_factor * zoom_factor;
		@result = null;
		float closest_dist = MAX_FLOAT;
		segment_index = -1;
		vertex_index = -1;
		control_point_index = 0;
		
		const int cp_i1 = curve.type == QuadraticBezier ? -1 : 0;
		const int cp_i2 = curve.type == QuadraticBezier ? 0 : 2;
		
		for(int i = 0; i < curve.vertex_count; i++)
		{
			CurveVertex@ p = curve.vertices[i];
			CurveVertex@ p2 = curve.vert(i, 1);
			
			for(int j = cp_i1; j < cp_i2; j++)
			{
				if(!curve.closed && (i == 0 && j == 0 || i == curve.vertex_count - 1 && j == 1))
					continue;
				
				CurveControlPoint@ cp = j == -1 ? p.quad_control_point
					: j == 0 ? p.cubic_control_point_1 : p.cubic_control_point_2;
				if(cp.type == Square)
					continue;
				const float cpx = p.x + cp.x - mouse.x;
				const float cpy = p.y + cp.y - mouse.y;
				
				const float dist = cpx * cpx + cpy * cpy;
				
				if(dist <= max_dist && dist < closest_dist)
				{
					closest_dist = dist;
					@result = cp;
					segment_index = j == 0 ? mod(i - 1, curve.vertex_count) : i;
					vertex_index = segment_index;
					control_point_index = j == -1 ? 1 : j == 0 ? 2 : 1;
				}
			}
		}
		
		return @result != null;
	}
	
	void start_drag_hover()
	{
		@drag_point = hover_point;
		@drag_vertex = hover_vertex;
		drag_is_vertex = hover_is_vertex;
		drag_segment_index = hover_segment_index;
		drag_vertex_index = hover_vertex_index;
		drag_control_point_index = hover_control_point_index;
	}
	
	void init_drag()
	{
		drag_ox = drag_point.x - mouse.x;
		drag_oy = drag_point.y - mouse.y;
		
		if(drag_is_vertex && curve.type == QuadraticBezier)
		{
			drag_cp_x = drag_vertex.x + drag_vertex.quad_control_point.x;
			drag_cp_y = drag_vertex.y + drag_vertex.quad_control_point.y;
		}
	}
	
	void stop_drag()
	{
		@drag_point = null;
		@drag_vertex = null;
		drag_segment_index = -1;
		drag_vertex_index = -1;
		drag_control_point_index = 0;
	}
	
	void recreate_spline()
	{
		curve.clear();
		
		if(is_rand)
		{
			srand(seed);
			for(int i = 0; i < 10; i++)
			{
				curve.add_vertex(200 + i * 100, rand_range(-100, 100));
			}
			seed++;
		}
		else
		{
			const float bx = 500;
			const float by = 0;
			curve.add_vertex(bx - 100, by - 100);
			curve.add_vertex(bx + 100, by - 100);
			curve.add_vertex(bx + 100, by + 100);
			curve.add_vertex(bx - 100, by + 000);
			curve.add_vertex(bx - 200, by + 200);
			curve.add_vertex(bx - 200, by + 000);
			
			//curve.add_vertex(bx - 300, by - 100);
			//curve.add_vertex(bx + 100, by - 100);
			//curve.add_vertex(bx + 100, by + 100);
			//curve.vertices[0].quad_control_point.set(bx+200, by-600);
			curve.vertices[0].quad_control_point.weight = 13.6;
		}
		
		//curve.init_bezier_control_points(true);
		//curve.vertices[2].quad_control_point.set(-100, 200);
		
		curve_changed = All;
	}
	
	uint get_curve_line_colour(const MultiCurve@ curve, const int segment_index, const int segment_max, const float t)
	{
		uint clr = curve.closed && segment_index == segment_max
			? 0xffff6569
			: segment_index % 2 == 0
				? 0xffffcc66 : 0xff222222;
		
		if(segment_alpha < 1)
		{
			clr = multiply_alpha(clr, segment_alpha);
		}
		
		return clr;
	}
	
	private void update_curve_precision()
	{
		MultiCuveSubdivisionSettings@  settings = curve.subdivision_settings;
		
		if(low_precision)
		{
			settings.angle_min = 10;
			settings.max_subdivisions = 1;
			settings.length_min = 1;
			settings.angle_max = 90;
		}
		else
		{
			settings.angle_min = 6;
			settings.max_subdivisions = 4;
			settings.length_min = 0;
			settings.angle_max = 65;
		}
		
		settings.max_stretch_factor = adaptive_stretch_factor ? 0.2 : 0.0;
	}
	
	private void display_text(const string txt, const int frames=1)
	{
		display_text(mouse.x, mouse.y - 5 * zoom_factor, txt, frames);
	}
	
	private void display_text_at_curve(const string txt, const int frames=1)
	{
		display_text((curve.x1 + curve.x2) * 0.5, curve.y1 - 25 * zoom_factor, txt, frames);
	}
	
	private void display_text(const float x, const float y, const string txt, const int time=1)
	{
		display_txt.text(txt);
		display_txt_x = x;
		display_txt_y = y;
		display_txt_timer = time;
	}
	
	private bool check(const int vk) { return !is_polling_keyboard && input.key_check_vk(vk); }
	private bool check_pressed(const int vk) { return !is_polling_keyboard && input.key_check_pressed_vk(vk); }
	private bool check_release(const int vk) { return !is_polling_keyboard && input.key_check_released_vk(vk); }
	private bool check_gvb(const int gvb) { return !is_polling_keyboard && input.key_check_gvb(gvb); }
	private bool check_pressed_gvb(const int gvb) { return !is_polling_keyboard && input.key_check_pressed_gvb(gvb); }
	private bool check_release_gvb(const int gvb) { return !is_polling_keyboard && input.key_check_released_gvb(gvb); }
	
}

enum EditState
{
	
	Idle,
	DragVertex,
	DragWeight,
	
}

class ClosestPointTest
{
	
	bool found;
	bool hover;
	float from_x, from_y;
	int i;
	float t;
	float x, y;
	float dx, dy;
	float nx, ny;
	float dist;
	
}

enum CurveChange
{
	
	None,
	Validate,
	All,
	
}
