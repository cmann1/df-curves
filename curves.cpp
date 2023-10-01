#include '../lib/std.cpp';
#include '../lib/drawing/common.cpp';
#include '../lib/input/Mouse.cpp';
#include '../lib/enums/GVB.cpp';
#include '../lib/enums/VK.cpp';
#include '../lib/utils/colour.cpp';

#include 'BaseCurve.cpp';

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
			switch(curve.type)
			{
				case Linear: curve.type = CatmullRom; break;
				case CatmullRom: curve.type = QuadraticBezier; break;
				case QuadraticBezier: curve.type = CubicBezier; break;
				case CubicBezier: curve.type = BSpline; break;
				case BSpline: curve.type = Linear; break;
			}
		}
		
		if(curve.end_controls == CurveEndControl::Manual)
		{
			if(mouse.left_down)
			{
				curve.control_point_start.x = g.mouse_x_world(0, 19);
				curve.control_point_start.y = g.mouse_y_world(0, 19);
			}
			if(mouse.right_down)
			{
				curve.control_point_end.x = g.mouse_x_world(0, 19);
				curve.control_point_end.y = g.mouse_y_world(0, 19);
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
			p.weight = curve.type == CubicBezier
				? map(sin((t * 4 + PI * 1.5 + i * 2 + 2) * 0.2), -1, 1, 0.0001, 6)
				: map(sin((t * 4 + PI * 1.5 + i * 2) * 0.4), -1, 1, 0.01, 12);
		}
		curve.invalidate();
		//for(uint i = 0; i < curve.cubic_bezier_control_points.length; i++)
		//{
		//	CurveVertex@ p = curve.cubic_bezier_control_points[i];
		//	p.weight = map(sin((t + PI * 1.5 + i) * 0.4), -1, 1, 0.0001, 6);
		//}
		curve.tension = map(sin((t * 4 + PI) * 0.5), -1, 1, 0.2, 2);
		//curve.vertices[0].tension = map(sin((t + PI + 1.2) * 1.3), -1, 1, 0.2, 10);
		
		t += speed * 0.25 * DT;
	}
	
	void editor_draw(float _)
	{
		curve.debug_draw(c, draw_zoom);
		
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
			curve.add_vertex(bx - 100, by + 100);
			//curve.add_vertex(bx - 300, by + 300);
			//curve.add_vertex(bx - 100, by - 100);
			//curve.add_vertex(bx + 100, by - 100);
			//curve.add_vertex(bx + 100, by + 100);
		}
		
		curve.init_bezier_control_points(true);
	}
	
}
