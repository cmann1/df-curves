#include '../lib/std.cpp';
#include '../lib/drawing/common.cpp';
#include '../lib/input/Mouse.cpp';
#include '../lib/enums/GVB.cpp';
#include '../lib/enums/VK.cpp';

#include 'BaseCurve.cpp';
#include 'BaseCurveDebug.cpp';

class script : BaseCurveDebugColourCallback
{
	
	[persist] float speed = 1;
	
	scene@ g;
	input_api@ input;
	editor_api@ editor;
	camera@ cam;
	canvas@ c;
	Mouse mouse(false);
	float zoom;
	float draw_zoom = 1;
	bool mouse_in_scene;
	bool space_down;
	
	EditState state = Idle;
	
	CurveControlPoint@ drag_point;
	float drag_ox, drag_oy;
	
	BaseCurve curve;
	BaseCurveDebug debug_draw;
	bool curve_changed;
	
	uint seed = 0;
	bool is_rand;
	
	float t = 0;
	
	script()
	{
		@g = get_scene();
		@input = get_input_api();
		@editor = get_editor_api();
		@c = create_canvas(false, 22, 22);
		mouse.use_input(input);
		
		@cam = get_active_camera();
		zoom = cam.editor_zoom();
		draw_zoom = 1 / zoom;
		
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
		zoom = cam.editor_zoom();
		draw_zoom = 1 / zoom;
		
		mouse_in_scene = !editor.mouse_in_gui() && editor.editor_tab() == 'Scripts';
		space_down = input.key_check_gvb(GVB::Space);
		const bool block_mouse = editor.mouse_in_gui() || space_down;
		mouse.step(block_mouse);
		
		switch(state)
		{
			case Idle: state_idle(); break;
			case DragVertex: state_drag_vertex(); break;
		}
		
		if(input.key_check_pressed_vk(VK::V))
		{
			calc_spline();
		}
		if(input.key_check_pressed_vk(VK::M))
		{
			curve.closed = !curve.closed;
			curve_changed = true;
		}
		if(input.key_check_pressed_vk(VK::K))
		{
			curve.b_spline_clamped = !curve.b_spline_clamped;
			curve_changed = true;
		}
		if(input.key_check_pressed_vk(VK::N))
		{
			is_rand = !is_rand;
			calc_spline();
		}
		if(input.key_check_pressed_vk(VK::OemComma))
		{
			curve.type = CurveType(mod(curve.type + (input.key_check_gvb(GVB::Shift) ? -1 : 1), BSpline + 1));
			curve_changed = true;
		}
		
		
		if(mouse.middle_press)
		{
			switch(curve.end_controls)
			{
				case CurveEndControl::AutomaticAngle: curve.end_controls = CurveEndControl::Automatic; break;
				case CurveEndControl::Automatic: curve.end_controls = CurveEndControl::Manual; break;
				case CurveEndControl::Manual: curve.end_controls = CurveEndControl::AutomaticAngle; break;
			}
		}
		
		for(uint i = 0; i < curve.vertices.length; i++)
		{
			CurveVertex@ p = curve.vertices[i];
			//p.type = Square;
			//p.weight = curve.type == CubicBezier
			//	? map(sin((t * 4 + PI * 1.5 + i * 2 + 2) * 0.2), -1, 1, 0.0001, 6)
			//	: map(sin((t * 4 + PI * 1.5 + i * 1) * 0.4), -1, 1, 0.01, 12);
			//p.tension = map(sin((t * 4 + PI + i) * 0.5), -1, 1, 0.2, 2);
			
			//p.quad_control_point.weight = map(sin((t * 4 + PI * 1.5 + i) * 0.4), -1, 1, 0.0001, 6);
			//p.cubic_control_point_1.weight = map(sin((t * 4 + PI * 1.5 + i * 2) * 0.4), -1, 1, 0.0001, 6);
			//p.cubic_control_point_2.weight = map(sin((t * 4 + PI * 1.5 + i * 2 + 1) * 0.4), -1, 1, 0.0001, 6);
		}
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
	
	void editor_draw(float _)
	{
		//curve.invalidate();
		//curve.update();
		debug_draw.draw(c, curve, draw_zoom);
		
		float x, y, nx, ny;
		curve.eval(abs(t % 2 - 1), x, y, nx, ny);
		//curve.calc(t % 1, x, y, nx, ny);
		draw_dot(g, 22, 22, x, y, 4 * draw_zoom, 0xffffffff, 45);
	}
	
	void state_idle()
	{
		if(mouse_in_scene && mouse.left_press)
		{
			if(get_vertex_at_mouse(drag_point))
			{
				drag_ox = drag_point.x - mouse.x;
				drag_oy = drag_point.y - mouse.y;
				state = DragVertex;
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
	
	bool get_vertex_at_mouse(CurveControlPoint@ &out result, const float size=5)
	{
		const float max_dist = size * size * draw_zoom * draw_zoom;
		@result = null;
		float closest_dist = MAX_FLOAT;
		
		const int start_i = curve.end_controls == CurveEndControl::Manual && curve.type == CatmullRom
			? -2 : 0;
		
		for(int i = start_i; i < curve.vertex_count; i++)
		{
			CurveControlPoint@ p = start_i >= 0 ?curve.vertices[i]
				: start_i == -1 ? curve.control_point_start : curve.control_point_end;
			const float dist = (p.x - mouse.x) * (p.x - mouse.x) + (p.y - mouse.y) * (p.y - mouse.y);
			
			if(dist <= max_dist && dist < closest_dist)
			{
				closest_dist = dist;
				@result = p;
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
			//curve.add_vertex(bx - 100, by - 100);
			//curve.add_vertex(bx + 100, by - 100);
			//curve.add_vertex(bx + 100, by + 100);
		}
		
		curve.init_bezier_control_points(true);
		curve.vertices[2].quad_control_point.set(-100, 200);
		
		curve_changed = true;
	}
	
	uint get_curve_line_colour(const BaseCurve@ curve, const float segment_t, const float max_t)
	{
		if(curve.closed && int(segment_t) == int(max_t) - 1)
			return 0xffff6569;
		
		return int(segment_t) % 2 == 0
			? 0xffffcc66 : 0xff222222;
	}
	
}

enum EditState
{
	
	Idle,
	DragVertex,
	
}
