/// Provides simple debug drawing for a `BaseCurve`.
/// Setting any width/length property to <= 0 will disable drawing of that component.
class BaseCurveDebug
{
	
	float control_point_size = 2;
	float control_point_line_width = 1;
	float line_width = 1;
	float normal_width = 1;
	float normal_length = 12;
	float outline_width = 1;
	float vertex_size = 3;
	float bounding_box_width = 3;
	
	uint line_clr = 0xffffffff;
	uint normal_clr = 0xaaff0000;
	uint outline_clr = 0x88999999;
	uint vertex_clr = 0xffff00ff;
	uint quad_cp_clr = 0xffff0000;
	uint cubic_cp1_clr = 0xffff0000;
	uint cubic_cp2_clr = 0xff0000ff;
	uint bounding_box_clr = 0x66002222;
	
	/// The precision used by `draw_curve`.
	int curve_segments = 15;
	
	private CurveVertex p0;
	private CurveVertex p3;
	
	/// Draws all components of the curve based on this instance's properties.
	void draw(canvas@ c, BaseCurve@ curve, const float draw_zoom=1)
	{
		draw_control_points(c, curve, draw_zoom);
		draw_outline(c, curve, draw_zoom);
		draw_curve(c, curve, draw_zoom);
		draw_vertices(c, curve, draw_zoom);
		draw_bounding_box(c, curve, draw_zoom);
	}
	
	/// Draws the control points and connecting lines.
	void draw_control_points(canvas@ c, BaseCurve@ curve, const float draw_zoom=1)
	{
		if(control_point_size <= 0)
			return;
		
		for(int i = 0; i < curve.vertex_count; i++)
		{
			CurveVertex@ p = curve.vertices[i];
			
			if(curve.type == CurveType::QuadraticBezier)
			{
				if(!curve.closed && i == curve.vertex_count - 1)
					continue;
				
				CurvePoint@ cp = p.quad_control_point;
				
				if(control_point_line_width > 0)
				{
					c.draw_line(p.x, p.y, p.x + cp.x, p.y + cp.y, control_point_line_width * draw_zoom, multiply_alpha(quad_cp_clr, 0.5));
					
					if(int(i) < curve.vertex_count - 1 || curve.closed)
					{
						CurveVertex@ p2 = curve.vert(i, 1);
						c.draw_line(p2.x, p2.y, p.x + cp.x, p.y + cp.y, 1 * draw_zoom, multiply_alpha(quad_cp_clr, 0.5));
					}
				}
				
				if(control_point_size > 0)
				{
					c.draw_rectangle(
						p.x + cp.x - control_point_size * draw_zoom, p.y + cp.y - control_point_size * draw_zoom,
						p.x + cp.x + control_point_size * draw_zoom, p.y + cp.y + control_point_size * draw_zoom,
						45, quad_cp_clr);
				}
			}
			else if(curve.type == CurveType::CubicBezier)
			{
				CurveControlPoint@ cp1 = p.cubic_control_point_1;
				CurveControlPoint@ cp2 = p.cubic_control_point_2;
				
				if(curve.closed || i > 0)
				{
					c.draw_line(p.x, p.y, p.x + cp1.x, p.y + cp1.y, control_point_line_width * draw_zoom, multiply_alpha(cubic_cp1_clr, 0.5));
					c.draw_rectangle(
						p.x + cp1.x - control_point_size * draw_zoom, p.y + cp1.y - control_point_size * draw_zoom,
						p.x + cp1.x + control_point_size * draw_zoom, p.y + cp1.y + control_point_size * draw_zoom,
						45, cubic_cp1_clr);
				}
				if(curve.closed || i < curve.vertex_count - 1)
				{
					c.draw_line(p.x, p.y, p.x + cp2.x, p.y + cp2.y, control_point_line_width * draw_zoom, multiply_alpha(cubic_cp2_clr, 0.5));
					c.draw_rectangle(
						p.x + cp2.x - control_point_size * draw_zoom, p.y + cp2.y - control_point_size * draw_zoom,
						p.x + cp2.x + control_point_size * draw_zoom, p.y + cp2.y + control_point_size * draw_zoom,
						45, cubic_cp1_clr);
				}
			}
		}
		
		if(curve.type == CatmullRom && !curve.closed && (control_point_line_width > 0 || control_point_size > 0))
		{
			if(curve.end_controls == CurveEndControl::Manual)
			{
				if(control_point_line_width > 0)
				{
					c.draw_line(
						curve.first_vertex.x, curve.first_vertex.y,
						curve.control_point_start.x, curve.control_point_start.y, control_point_line_width * draw_zoom,
						multiply_alpha(cubic_cp1_clr, 0.85));
					c.draw_line(
						curve.last_vertex.x, curve.last_vertex.y,
						curve.control_point_end.x, curve.control_point_end.y, control_point_line_width * draw_zoom,
						multiply_alpha(cubic_cp1_clr, 0.85));
				}
				
				if(control_point_size > 0)
				{
					c.draw_rectangle(
						curve.control_point_start.x - control_point_size * draw_zoom,
						curve.control_point_start.y - control_point_size * draw_zoom,
						curve.control_point_start.x + control_point_size * draw_zoom,
						curve.control_point_start.y + control_point_size * draw_zoom,
						45, multiply_alpha(cubic_cp1_clr, 0.5));
					c.draw_rectangle(
						curve.control_point_end.x - control_point_size * draw_zoom,
						curve.control_point_end.y - control_point_size * draw_zoom,
						curve.control_point_end.x + control_point_size * draw_zoom,
						curve.control_point_end.y + control_point_size * draw_zoom,
						45, multiply_alpha(cubic_cp1_clr, 0.5));
				}
			}
			else
			{
				curve.get_auto_control_start(p0, curve.end_controls);
				curve.get_auto_control_end(p3, curve.end_controls);
				
				if(control_point_line_width > 0)
				{
					c.draw_line(
						curve.first_vertex.x, curve.first_vertex.y,
						p0.x, p0.y, control_point_line_width * draw_zoom,
						multiply_alpha(cubic_cp1_clr, 0.5));
					c.draw_line(
						curve.last_vertex.x, curve.last_vertex.y,
						p3.x, p3.y, control_point_line_width * draw_zoom,
						multiply_alpha(cubic_cp1_clr, 0.5));
				}
				
				if(control_point_size > 0)
				{
					c.draw_rectangle(
						p0.x - control_point_size * draw_zoom, p0.y - control_point_size * draw_zoom,
						p0.x + control_point_size * draw_zoom, p0.y + control_point_size * draw_zoom,
						45, multiply_alpha(cubic_cp1_clr, 0.5));
					c.draw_rectangle(
						p3.x - control_point_size * draw_zoom, p3.y - control_point_size * draw_zoom,
						p3.x + control_point_size * draw_zoom, p3.y + control_point_size * draw_zoom,
						45, multiply_alpha(cubic_cp1_clr, 0.5));
				}
			}
		}
	}
	
	/// Draws the skeleton, ie. lines connecting each vertex.
	void draw_outline(canvas@ c, BaseCurve@ curve, const float draw_zoom=1)
	{
		if(outline_width <= 0 || curve.vertex_count <= 0 || curve.type == CurveType::Linear)
			return;
		
		CurveVertex@ p1 = curve.vertices[0];
		
		for(int i = 1; i < curve.vertex_count; i++)
		{
			CurveVertex@ p2 = curve.vertices[i];
			
			c.draw_line(p1.x, p1.y, p2.x, p2.y, outline_width * draw_zoom, outline_clr);
			
			@p1 = p2;
		}
	}
	
	/// Draws the curve.
	void draw_curve(canvas@ c, BaseCurve@ curve, const float draw_zoom=1)
	{
		const bool draw_curve = line_width > 0;
		const bool draw_normal = normal_width > 0 && normal_length > 0;
		
		if(curve_segments <= 0 || !draw_curve && draw_normal)
			return;
		
		float x1 = 0;
		float y1 = 0;
		const int count = curve_segments * (curve.vertex_count - (curve.closed ? 0 : 1));
		const EvalReturnType eval_type = draw_curve && draw_normal || draw_normal
			? EvalReturnType::Both : EvalReturnType::Point;
		
		for(int i = 0; i <= count; i++)
		{
			const float t = float(i) / count;
			float x2, y2, nx, ny;
			curve.eval(t, x2, y2, nx, ny, eval_type);
			
			if(draw_normal)
			{
				const float l = normal_length * draw_zoom;
				c.draw_line(x2, y2, x2 + nx * l, y2 + ny * l, normal_width * draw_zoom, normal_clr);
			}
			
			if(i > 0 && draw_curve)
			{
				c.draw_line(x1, y1, x2, y2, line_width * draw_zoom, line_clr);
			}
			
			x1 = x2;
			y1 = y2;
		}
	}
	
	/// Draws the actual curve.
	void draw_vertices(canvas@ c, BaseCurve@ curve, const float draw_zoom=1)
	{
		if(vertex_size <= 0)
			return;
		
		for(int i = 0; i < curve.vertex_count; i++)
		{
			CurveVertex@ p = curve.vertices[i];
			c.draw_rectangle(
				p.x - vertex_size * draw_zoom, p.y - vertex_size * draw_zoom,
				p.x + vertex_size * draw_zoom, p.y + vertex_size * draw_zoom,
				45, vertex_clr);
		}
	}
	
	/// Draws the bounding box of the curve.
	void draw_bounding_box(canvas@ c, BaseCurve@ curve, const float draw_zoom=1)
	{
		if(bounding_box_width <= 0)
			return;
		
		// Left
		c.draw_rectangle(
			curve.x1 - bounding_box_width * draw_zoom, curve.y1,
			curve.x1, curve.y2,
			0, bounding_box_clr);
		// Right
		c.draw_rectangle(
			curve.x2, curve.y1,
			curve.x2 + bounding_box_width * draw_zoom, curve.y2,
			0, bounding_box_clr);
		// Top
		c.draw_rectangle(
			curve.x1 - bounding_box_width * draw_zoom, curve.y1 - bounding_box_width * draw_zoom,
			curve.x2 + bounding_box_width * draw_zoom, curve.y1,
			0, bounding_box_clr);
		// Bottom
		c.draw_rectangle(
			curve.x1 - bounding_box_width * draw_zoom, curve.y2,
			curve.x2 + bounding_box_width * draw_zoom, curve.y2 + bounding_box_width * draw_zoom,
			0, bounding_box_clr);
	}
	
}
