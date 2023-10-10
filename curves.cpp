#include '../lib/std.cpp';
#include '../lib/drawing/common.cpp';
#include '../lib/input/Mouse.cpp';
#include '../lib/enums/GVB.cpp';
#include '../lib/enums/VK.cpp';

#include 'MultiCurve.cpp';
#include 'MultiCurveDebug.cpp';

class script : MultiCurveDebugColourCallback
{
	
	[persist] float speed = 1;
	
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
	
	EditState state = Idle;
	
	CurveControlPoint@ drag_point;
	float drag_ox, drag_oy;
	
	MultiCurve curve;
	MultiCurveDebug debug_draw;
	bool curve_changed;
	
	uint seed = 0;
	bool is_rand;
	
	float t = 0;
	
	textfield@ display_txt;
	float display_txt_timer = -1;
	float display_txt_x, display_txt_y;
	
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
		
		curve.type = BSpline;
		curve.closed = true;
		
		calc_spline();
		
		curve.invalidate();
		curve.validate();
		curve_changed = false;
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
		debug_draw.adaptive_angle = map_clamped(zoom_factor, 0.1, 0.75, 2, 5);
		
		mouse_in_scene = !editor.mouse_in_gui() && editor.editor_tab() == 'Scripts';
		const bool block_mouse = editor.mouse_in_gui() || space_down;
		mouse.step(block_mouse);
		
		space_down = input.key_check_gvb(GVB::Space);
		ctrl_down = input.key_check_gvb(GVB::Control);
		shift_down = input.key_check_gvb(GVB::Shift);
		alt_down = input.key_check_gvb(GVB::Alt);
		is_polling_keyboard = input.is_polling_keyboard();
		
		switch(state)
		{
			case Idle: state_idle(); break;
			case DragVertex: state_drag_vertex(); break;
			case DragWeight: state_drag_weight(); break;
		}
		
		if(check_pressed(VK::V))
		{
			calc_spline();
		}
		if(check_pressed(VK::M))
		{
			curve.closed = !curve.closed;
			curve_changed = true;
		}
		if(check_pressed(VK::K) && curve.type == BSpline)
		{
			curve.b_spline_clamped = !curve.b_spline_clamped;
			curve_changed = true;
		}
		if(check_pressed(VK::N))
		{
			is_rand = !is_rand;
			calc_spline();
		}
		if(check_pressed(VK::OemComma))
		{
			curve.type = CurveType(mod(curve.type + (input.key_check_gvb(GVB::Shift) ? -1 : 1), BSpline + 1));
			curve.invalidate();
			curve.validate();
			display_text_at_curve('Curve type: ' + get_curve_name(curve.type), 30);
		}
		if(check_pressed(VK::L) && curve.type == CatmullRom)
		{
			switch(curve.end_controls)
			{
				case CurveEndControl::AutomaticAngle: curve.end_controls = CurveEndControl::Automatic; break;
				case CurveEndControl::Automatic: curve.end_controls = CurveEndControl::Manual; break;
				case CurveEndControl::Manual: curve.end_controls = CurveEndControl::AutomaticAngle; break;
			}
			curve_changed = true;
		}
		
		if(mouse_in_scene && mouse.scroll != 0)
		{
			switch(curve.type)
			{
				case CatmullRom:
					curve.tension = clamp(curve.tension - mouse.scroll * 0.1, 0.25, 30.0);
					display_text_at_curve('Tension: ' + str(curve.tension), 25);
					curve_changed = true;
					break;
				case BSpline:
					curve.b_spline_degree = clamp(curve.b_spline_degree - mouse.scroll, 2, 7);
					display_text_at_curve('B-Spline degree: ' + curve.b_spline_degree, 25);
					curve_changed = true;
					break;
			}
		}
		
		if(mouse_in_scene && mouse.middle_press)
		{
			curve.tension = 1;
			curve_changed = true;
		}
		
		//for(uint i = 0; i < curve.vertices.length; i++)
		//{
		//	CurveVertex@ p = curve.vertices[i];
		//	p.type = Square;
		//	p.weight = curve.type == CubicBezier
		//		? map(sin((t * 4 + PI * 1.5 + i * 2 + 2) * 0.2), -1, 1, 0.0001, 6)
		//		: map(sin((t * 4 + PI * 1.5 + i * 1) * 0.4), -1, 1, 0.01, 12);
		//	p.tension = map(sin((t * 4 + PI + i) * 0.5), -1, 1, 0.2, 2);
		//	
		//	p.quad_control_point.weight = map(sin((t * 4 + PI * 1.5 + i) * 0.4), -1, 1, 0.0001, 6);
		//	p.cubic_control_point_1.weight = map(sin((t * 4 + PI * 1.5 + i * 2) * 0.4), -1, 1, 0.0001, 6);
		//	p.cubic_control_point_2.weight = map(sin((t * 4 + PI * 1.5 + i * 2 + 1) * 0.4), -1, 1, 0.0001, 6);
		//}
		//curve.vertices[1].cubic_control_point_1.weight = map(sin((t * 8 + PI * 1.5) * 0.2), -1, 1, 0.01, 6);
		//curve.vertices[1].weight = map(sin((t * 8 + PI * 1.5) * 0.2), -1, 1, 0.01, 2);
		//curve.tension = map(sin((t * 4 + PI) * 0.5), -1, 1, 0.2, 1);
		//curve.vertices[0].tension = map(sin((t + PI + 1.2) * 1.3), -1, 1, 0.2, 10);
		
		if(curve_changed)
		{
			curve.invalidate();
			curve.validate();
			curve_changed = false;
		}
		
		t += speed * 0.25 * DT;
	}
	
	void editor_draw(float)
	{
		//debug_draw.draw(c, curve, zoom_factor);
		
		debug_draw.draw_outline(c, curve, zoom_factor);
		debug_draw.draw_control_points(c, curve, zoom_factor);
		debug_draw.draw_arch_lengths(c, curve, zoom_factor);
		debug_draw.draw_vertices(c, curve, zoom_factor);
		
		float x, y, nx, ny;
		curve.eval(-1, abs(t % 2 - 1), x, y, nx, ny);
		//curve.eval(t % 1, x, y, nx, ny);
		draw_dot(g, 22, 22, x, y, 4 * zoom_factor, 0xffffffff, 45);
		
		if(display_txt_timer > -1)
		{
			display_txt.colour(display_txt_timer > 0 ? 0xffffffff : multiply_alpha(0xffffffff, 1 + display_txt_timer));
			display_txt.draw_world(22, 22, display_txt_x, display_txt_y, zoom_factor, zoom_factor, 0);
		}
	}
	
	void state_idle()
	{
		if(mouse_in_scene && mouse.right_double_click && alt_down)
		{
			if(get_vertex_at_mouse(drag_point) || get_control_point_at_mouse(drag_point))
			{
				if(curve.type != CatmullRom)
				{
					drag_point.weight = 1;
				}
				else
				{
					CurveVertex@ v = cast<CurveVertex@>(drag_point);
					if(@v != null)
					{
						v.tension = 1;
					}
				}
				@drag_point = null;
				curve_changed = true;
			}
			return;
		}
		
		if(mouse_in_scene && mouse.left_press)
		{
			if(get_vertex_at_mouse(drag_point) || get_control_point_at_mouse(drag_point))
			{
				drag_ox = drag_point.x - mouse.x;
				drag_oy = drag_point.y - mouse.y;
				state = DragVertex;
				return;
			}
		}
		
		if(mouse_in_scene && mouse.right_press && alt_down)
		{
			if(get_vertex_at_mouse(drag_point) || get_control_point_at_mouse(drag_point))
			{
				drag_ox = mouse.x;
				drag_oy = drag_point.weight;
				state = DragWeight;
				return;
			}
		}
	}
	
	void state_drag_vertex()
	{
		if(!mouse.left_down)
		{
			@drag_point = null;
			state = Idle;
			return;
		}
		
		if(mouse.moved)
		{
			drag_point.x = mouse.x + drag_ox;
			drag_point.y = mouse.y + drag_oy;
			curve_changed = true;
		}
	}
	
	void state_drag_weight()
	{
		if(!mouse.right_down)
		{
			@drag_point = null;
			state = Idle;
			return;
		}
		
		CurveVertex@ v = cast<CurveVertex@>(drag_point);
		
		if(mouse.moved)
		{
			if(curve.type == CatmullRom)
			{
				if(@v != null)
				{
					v.tension = clamp(drag_oy + (mouse.x - drag_ox) / zoom_factor * 0.2, 0.25, 30.0);
					display_txt.text('Tension: ' + str(v.tension));
				}
			}
			else
			{
				drag_point.weight = clamp(drag_oy + (mouse.x - drag_ox) / zoom_factor * 0.2, 0.05, 60.0);
				display_txt.text('Weight: ' + str(drag_point.weight));
			}
			
			curve_changed = true;
		}
		
		if(curve.type == CatmullRom)
		{
			if(@v != null)
			{
				display_txt.text('Tension: ' + str(v.tension));
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
	
	bool get_vertex_at_mouse(CurveControlPoint@ &out result, const float size=5)
	{
		const float max_dist = size * zoom_factor;
		@result = null;
		float closest_dist = MAX_FLOAT;
		
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
			}
		}
		
		return @result != null;
	}
	
	bool get_control_point_at_mouse(CurveControlPoint@ &out result, const float size=4)
	{
		if(curve.type != QuadraticBezier && curve.type != CubicBezier)
			return false;
		
		const float max_dist = size * size * zoom_factor * zoom_factor;
		@result = null;
		float closest_dist = MAX_FLOAT;
		
		const int cp_i1 = curve.type == QuadraticBezier ? -1 : 0;
		const int cp_i2 = curve.type == QuadraticBezier ? 0 : 2;
		
		for(int i = 0; i < curve.vertex_count; i++)
		{
			CurveVertex@ p = curve.vertices[i];
			
			for(int j = cp_i1; j < cp_i2; j++)
			{
				if(!curve.closed && (i == 0 && j == 0 || i == curve.vertex_count - 1 && j == 1))
					continue;
				
				CurveControlPoint@ cp = j == -1 ? p.quad_control_point
					: j == 0 ? p.cubic_control_point_1 : p.cubic_control_point_2;
				const float cpx = (j != -1 ? p.x : 0) + cp.x - mouse.x;
				const float cpy = (j != -1 ? p.y : 0) + cp.y - mouse.y;
				
				const float dist = cpx * cpx + cpy * cpy;
				
				if(dist <= max_dist && dist < closest_dist)
				{
					closest_dist = dist;
					@result = cp;
				}
			}
		}
		
		return @result != null;
	}
	
	void calc_spline()
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
			
			//curve.add_vertex(bx - 100, by - 100);
			//curve.add_vertex(bx + 100, by - 100);
			//curve.add_vertex(bx + 100, by + 100);
			//curve.vertices[0].quad_control_point.set(bx+200, by-600);
			curve.vertices[0].quad_control_point.weight = 13.6;
		}
		
		curve.init_bezier_control_points(true);
		//curve.vertices[2].quad_control_point.set(-100, 200);
		
		curve_changed = true;
	}
	
	uint get_curve_line_colour(const MultiCurve@ curve, const int segment_index, const int segment_max, const float t)
	{
		if(curve.closed && segment_index == segment_max)
			return 0xffff6569;
		
		return segment_index % 2 == 0
			? 0xffffcc66 : 0xff222222;
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
	
	private string get_curve_name(const CurveType type)
	{
		switch(curve.type)
		{
			case Linear: return 'Linear';
			case QuadraticBezier: return 'QuadraticBezier';
			case CubicBezier: return 'CubicBezier';
			case CatmullRom: return 'CatmullRom';
			case BSpline: return 'BSpline';
		}
		
		return 'Unknown';
	}
	
}

enum EditState
{
	
	Idle,
	DragVertex,
	DragWeight,
	
}
