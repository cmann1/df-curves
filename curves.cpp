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
	Mouse mouse;
	float zoom;
	float view_mult;
	
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
		mouse.use_input(input);
		
		curve.type = CatmullRom;
		calc_spline();
		
		@cam = get_active_camera();
	}
	
	void editor_step()
	{
		zoom = cam.editor_zoom();
		view_mult = 1 / zoom;
		
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
				? map(sin((t + PI * 1.5 + i * 2 + 2) * 0.2), -1, 1, 0.0001, 6)
				: map(sin((t + PI * 1.5) * 0.4), -1, 1, -0.65, 6);
		}
		for(uint i = 0; i < curve.cubic_bezier_control_points.length; i++)
		{
			CurveVertex@ p = curve.cubic_bezier_control_points[i];
			p.weight = map(sin((t + PI * 1.5 + i) * 0.4), -1, 1, 0.0001, 6);
		}
		curve.tension = map(sin((t + PI) * 0.5), -1, 1, 0.2, 2);
		curve.vertices[0].tension = map(sin((t + PI + 1.2) * 1.3), -1, 1, 0.2, 10);
		
		t += speed * DT;
	}
	
	void editor_draw(float _)
	{
		
		if(curve.type == QuadraticBezier)
		{
			for(uint i = 0; i < curve.quadratic_bezier_control_points.length; i++)
			{
				if(!curve.closed && i == curve.quadratic_bezier_control_points.length - 1)
					continue;
				
				const uint clr = 0xffff0000;
				CurveVertex@ p = curve.vertices[i];
				Point@ cp = @curve.quadratic_bezier_control_points[i];
				g.draw_line_world(22, 22, p.x, p.y, p.x + cp.x, p.y + cp.y, 1 * view_mult, multiply_alpha(clr, 0.5));
				if(int(i) < curve.vertex_count - 1 || curve.closed)
				{
					CurveVertex@ p2 = curve.vert(i, 1);
					g.draw_line_world(22, 22, p2.x, p2.y, p.x + cp.x, p.y + cp.y, 1 * view_mult, multiply_alpha(clr, 0.5));
				}
				draw_dot(g, 22, 22, p.x + cp.x, p.y + cp.y, 2 * view_mult, clr, 45);
			}
		}
		else if(curve.type == CubicBezier)
		{
			for(uint i = 0; i < curve.cubic_bezier_control_points.length; i++)
			{
				if(!curve.closed && (i == 0 || i == curve.cubic_bezier_control_points.length - 1))
					continue;
				
				const uint clr = i % 2 == 0 ? 0xffff0000 : 0xff0000ff;
				CurveVertex@ p = curve.vertices[i / 2];
				Point@ cp = @curve.cubic_bezier_control_points[i];
				g.draw_line_world(22, 22, p.x, p.y, p.x + cp.x, p.y + cp.y, 1 * view_mult, multiply_alpha(clr, 0.5));
				draw_dot(g, 22, 22, p.x + cp.x, p.y + cp.y, 2 * view_mult, clr, 45);
			}
		}
		
		float x1 = 0;
		float y1 = 0;
		const int count = 18 * (curve.vertices.length - (curve.closed ? 0 : 1));
		for(int i = 0; i <= count; i++)
		{
			const float t = float(i) / count;
			float x2, y2, nx, ny;
			curve.calc(t, x2, y2, nx, ny);
			
			const float l = 12 * view_mult;
			g.draw_line_world(22, 22, x2 - nx * l * 0.0, y2 - ny * l * 0.0, x2 + nx * l, y2 + ny * l, 1 * view_mult, 0xaaff0000);
			
			if(i > 0)
			{
				g.draw_line_world(22, 22, x1, y1, x2, y2, 1 * view_mult, 0xffffffff);
			}
			
			x1 = x2;
			y1 = y2;
		}
		
		for(uint i = 0; i < curve.vertices.length; i++)
		{
			draw_dot(g, 22, 22, curve.vertices[i].x, curve.vertices[i].y, 3 * view_mult, 0xffff00ff, 45);
		}
		
		if(curve.type == CatmullRom && !curve.closed)
		{
			if(curve.end_controls == CurveEndControl::Manual)
			{
				draw_dot(g, 22, 22, curve.control_point_start.x, curve.control_point_start.y, 3 * view_mult, 0xffff00ff, 45);
				draw_dot(g, 22, 22, curve.control_point_end.x, curve.control_point_end.y, 3 * view_mult, 0xffff00ff, 45);
				g.draw_line_world(22, 22, curve.first_vertex.x, curve.first_vertex.y, curve.control_point_start.x, curve.control_point_start.y, 1 * view_mult, 0x99ffffff);
				g.draw_line_world(22, 22, curve.last_vertex.x, curve.last_vertex.y, curve.control_point_end.x, curve.control_point_end.y, 1 * view_mult, 0x99ffffff);
			}
			else
			{
				CurveVertex p;
				curve.get_auto_control_start(p, curve.end_controls);
				draw_dot(g, 22, 22, p.x, p.y, 2 * view_mult, 0xaaff00ff, 45);
				g.draw_line_world(22, 22, curve.first_vertex.x, curve.first_vertex.y, p.x, p.y, 1 * view_mult, 0x55ffffff);
				curve.get_auto_control_end(p, curve.end_controls);
				draw_dot(g, 22, 22, p.x, p.y, 2 * view_mult, 0xaaff00ff, 45);
				g.draw_line_world(22, 22, curve.last_vertex.x, curve.last_vertex.y, p.x, p.y, 1 * view_mult, 0x55ffffff);
			}
		}
		
		float x, y, nx, ny;
		curve.calc(abs((t * 0.25) % 2 - 1), x, y, nx, ny);
		draw_dot(g, 22, 22, x, y, 4 * view_mult, 0xffffffff, 45);
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
			curve.add_vertex(bx - 100, by + 100);
		}
		
		curve.calc_bezier_control_points(true);
	}
	
}
