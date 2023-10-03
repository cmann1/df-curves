#include '../lib/std.cpp';
#include '../lib/drawing/common.cpp';
#include '../lib/input/Mouse.cpp';
#include '../lib/enums/GVB.cpp';
#include '../lib/enums/VK.cpp';
#include '../lib/utils/colour.cpp';

#include 'BaseCurve.cpp';
#include 'BaseCurveDebug.cpp';

class script
{
	
	scene@ g;
	input_api@ input;
	editor_api@ editor;
	camera@ cam;
	canvas@ c;
	Mouse mouse(false);
	float zoom;
	float draw_zoom = 1;
	
	[persist] float speed = 1;
	BaseCurve curve;
	BaseCurveDebug debug_draw;
	
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
		
		debug_draw.line_alt_clr = 0xff555555;
		
		curve.type = BSpline;
		calc_spline();
		
		@cam = get_active_camera();
	}
	
	void editor_step()
	{
		zoom = cam.editor_zoom();
		draw_zoom = 1 / zoom;
		
		const bool space = input.key_check_gvb(GVB::Space);
		const bool block_mouse = editor.mouse_in_gui() || space;
		mouse.step(block_mouse);
		
		if(input.key_check_pressed_vk(VK::V))
		{
			calc_spline();
		}
		if(input.key_check_pressed_vk(VK::M))
		{
			curve.closed = !curve.closed;
		}
		
		if(input.key_check_pressed_vk(VK::N))
		{
			is_rand = !is_rand;
			calc_spline();
		}
		if(input.key_check_pressed_vk(VK::OemComma))
		{
			curve.type = CurveType(mod(curve.type + (input.key_check_gvb(GVB::Shift) ? -1 : 1), BSpline + 1));
		}
		
		if(curve.end_controls == CurveEndControl::Manual && curve.type == CatmullRom)
		{
			if(mouse.left_down)
			{
				curve.control_point_start.x = mouse.x;
				curve.control_point_start.y = mouse.y;
			}
			if(mouse.right_down)
			{
				curve.control_point_end.x = mouse.x;
				curve.control_point_end.y = mouse.y;
			}
		}
		else
		{
			if(mouse.left_down)
			{
				curve.vertices[0].set(mouse.x, mouse.y);
			}
			if(mouse.right_down)
			{
				curve.vertices[1].set(mouse.x, mouse.y);
			}
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
		
		curve.invalidate();
		curve.update();
		
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
	
	void calc_spline()
	{
		curve.clear();
		//curve.closed = false;
		
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
			//curve.add_vertex(bx - 300, by + 300);
			//curve.add_vertex(bx - 100, by - 100);
			//curve.add_vertex(bx + 100, by - 100);
			//curve.add_vertex(bx + 100, by + 100);
		}
		
		curve.init_bezier_control_points(true);
		curve.vertices[2].quad_control_point.set(-100, 200);
		curve.update();
	}
	
}
