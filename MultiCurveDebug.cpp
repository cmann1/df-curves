#include '../lib/utils/colour.cpp';

/** Provides simple debug drawing for a `MultiCurve`.
  * Setting any width/length property to <= 0 will disable drawing of that component. */
class MultiCurveDebug
{
	
	float control_point_size = 2;
	float control_point_line_width = 1;
	float line_width = 2;
	float normal_width = 1;
	float normal_length = 14;
	/** A length multiplier used when rendering normals within subdivided segments. */
	float normal_multiplier_adaptive = 0.65;
	float outline_width = 1;
	float vertex_size = 3;
	float segment_bounding_box_width = 0;
	float bounding_box_width = 3;
	float segment_index_label_scale = 1;
	float segment_index_label_offset = 10;
	float hover_vertex_size_mult = 1;
	float hover_control_point_size_mult = 1.25;
	float hover_outline_thickness = 2;
	float hover_outline_blend = 0.65;
	
	uint line_clr = 0xff222222;
	uint normal_clr = 0xccff0000;
	/** A colour used when rendering normals within subdivided segments.
	  * Set to zero to use the standard normal colour. */
	uint normal_adaptive_clr = 0xaaee7700;
	uint outline_clr = 0x88999999;
	uint vertex_clr = 0xffc487f8;
	uint quad_cp_clr = 0xffff0000;
	uint cubic_cp1_clr = 0xffff3322;
	uint cubic_cp2_clr = 0xff5599ff;
	uint segment_bounding_box_clr = 0x44002222;
	uint bounding_box_clr = 0x66002222;
	uint segment_index_label_clr = 0x99ffffff;
	uint hover_outline_clr = 0x77000000;
	
	int hovered_vertex_index = -1;
	int hovered_control_point_index = 0;
	
	/** If set, allows choosing the curve line colour based on the segment index and absolute t value. */
	MultiCurveDebugColourCallback@ segment_colour_callback;
	
	/** The precision used by `draw_curve`. */
	int curve_segments = 15;
	
	/** If this and `adaptive_max_subdivisions` are greater than zero, the curve will be subdivided when the angle in degrees between
	  * the start and the end of a segment is greater than this value. */
	float adaptive_angle = 0;
	/** Used to limit the subdivisions when a segment's angle is above the adaptive threshold. */
	int adaptive_max_subdivisions = 0;
	/** Stops subdividing when the length of the segments is lower than this. */
	float adaptive_min_length = 0;
	
	/** If true anything outside of the clip bounds will not be drawn.
	  * Curve bounding must be calculated for this to work correctly. */
	bool clip;
	/** The corner positions defining the clipping rectangle. */
	float clip_x1, clip_y1;
	float clip_x2, clip_y2;
	
	private CurveVertex p0;
	private CurveVertex p3;
	
	private float _clip_x1, _clip_y1;
	private float _clip_x2, _clip_y2;
	
	private textfield@ tf;
	
	MultiCurveDebug()
	{
		@tf = create_textfield();
		tf.align_horizontal(0);
		tf.align_vertical(1);
		tf.set_font('envy_bold', 20);
	}
	
	/** Draws all components of the curve based on this instance's properties. */
	void draw(canvas@ c, MultiCurve@ curve, const float zoom_factor=1)
	{
		draw_bounding_box(c, curve, zoom_factor);
		draw_control_points(c, curve, zoom_factor);
		draw_outline(c, curve, zoom_factor);
		draw_curve(c, curve, zoom_factor);
		draw_vertices(c, curve, zoom_factor);
		draw_segment_labels(c, curve, zoom_factor);
		draw_hovered(c, curve, zoom_factor);
	}
	
	/** Draws the control points and connecting lines. */
	void draw_control_points(canvas@ c, MultiCurve@ curve, const float zoom_factor=1)
	{
		if(control_point_size <= 0)
			return;
		
		const float cpw = control_point_line_width * zoom_factor;
		const float cps = control_point_size * zoom_factor;
		
		if(!check_clip(curve, max(cpw, sqrt(cps * cps + cps * cps)), 1, true))
			return;
		
		for(int i = 0; i < curve.vertex_count; i++)
		{
			CurveVertex@ p = curve.vertices[i];
			
			if(curve.type == CurveType::QuadraticBezier)
			{
				CurveVertex@ p2 = curve.vert(i, 1);
				
				if(!curve.closed && i == curve.vertex_count - 1)
					continue;
				
				CurveControlPoint@ cp = p.quad_control_point;
				if(cp.type == Square)
					continue;
				
				if(clip)
				{
					const float x1 = min(min(p.x, p.x + cp.x), p2.x);
					const float y1 = min(min(p.y, p.y + cp.y), p2.y);
					const float x2 = max(max(p.x, p.x + cp.x), p2.x);
					const float y2 = max(max(p.y, p.y + cp.y), p2.y);
					if(x1 > _clip_x2 || x2 < _clip_x1 || y1 > _clip_y2 || y2 < _clip_y1)
						continue;
				}
				
				if(control_point_line_width > 0)
				{
					c.draw_line(p.x, p.y, p.x + cp.x, p.y + cp.y, cpw, multiply_alpha(quad_cp_clr, 0.5));
					
					if(int(i) < curve.vertex_count - 1 || curve.closed)
					{
						c.draw_line(p2.x, p2.y, p.x + cp.x, p.y + cp.y, cpw, multiply_alpha(quad_cp_clr, 0.5));
					}
				}
				
				if(control_point_size > 0 && (i != hovered_vertex_index || hovered_control_point_index != 1))
				{
					c.draw_rectangle(
						p.x + cp.x - cps, p.y + cp.y - cps,
						p.x + cp.x + cps, p.y + cp.y + cps,
						45, quad_cp_clr);
				}
			}
			else if(curve.type == CurveType::CubicBezier)
			{
				CurveControlPoint@ cp1 = p.cubic_control_point_1;
				CurveControlPoint@ cp2 = p.cubic_control_point_2;
				
				if(clip)
				{
					const float x1 = p.x + cp1.x < p.x + cp2.x ? p.x + cp1.x : p.x + cp2.x;
					const float y1 = p.y + cp1.y < p.y + cp2.y ? p.y + cp1.y : p.y + cp2.y;
					const float x2 = p.x + cp1.x > p.x + cp2.x ? p.x + cp1.x : p.x + cp2.x;
					const float y2 = p.y + cp1.y > p.y + cp2.y ? p.y + cp1.y : p.y + cp2.y;
					if(x1 > _clip_x2 || x2 < _clip_x1 || y1 > _clip_y2 || y2 < _clip_y1)
						continue;
				}
				
				if(cp1.type != Square)
				{
					c.draw_line(p.x, p.y, p.x + cp1.x, p.y + cp1.y, cpw, multiply_alpha(cubic_cp1_clr, 0.5));
					
					if(hovered_control_point_index != 2 || mod(i - 1, curve.vertex_count) != hovered_vertex_index)
					{
						c.draw_rectangle(
							p.x + cp1.x - cps, p.y + cp1.y - cps,
							p.x + cp1.x + cps, p.y + cp1.y + cps,
							45, cubic_cp1_clr);
					}
				}
				if(cp2.type != Square)
				{
					c.draw_line(p.x, p.y, p.x + cp2.x, p.y + cp2.y, cpw, multiply_alpha(cubic_cp2_clr, 0.5));
					
					if(hovered_control_point_index != 1 || i != hovered_vertex_index)
					{
						c.draw_rectangle(
							p.x + cp2.x - cps, p.y + cp2.y - cps,
							p.x + cp2.x + cps, p.y + cp2.y + cps,
							45, cubic_cp2_clr);
					}
				}
			}
		}
		
		if(curve.type == CatmullRom && !curve.closed && (control_point_line_width > 0 || control_point_size > 0))
		{
			if(curve.end_controls == CurveEndControl::Manual)
			{
				const float x1 = curve.first_vertex.x + curve.control_point_start.x;
				const float y1 = curve.first_vertex.y + curve.control_point_start.y;
				const float x2 = curve.last_vertex.x + curve.control_point_end.x;
				const float y2 = curve.last_vertex.y + curve.control_point_end.y;
				
				if(control_point_line_width > 0)
				{
					c.draw_line(
						curve.first_vertex.x, curve.first_vertex.y,
						x1, y1, cpw,
						multiply_alpha(outline_clr, 0.85));
					c.draw_line(
						curve.last_vertex.x, curve.last_vertex.y,
						x2, y2, cpw,
						multiply_alpha(outline_clr, 0.85));
				}
				
				if(control_point_size > 0)
				{
					c.draw_rectangle(
						x1 - cps, y1 - cps,
						x1 + cps, y1 + cps,
						45, multiply_alpha(cubic_cp1_clr, 0.5));
					c.draw_rectangle(
						x2 - cps, y2 - cps,
						x2 + cps, y2 + cps,
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
						p0.x, p0.y, cpw,
						multiply_alpha(cubic_cp1_clr, 0.5));
					c.draw_line(
						curve.last_vertex.x, curve.last_vertex.y,
						p3.x, p3.y, cpw,
						multiply_alpha(cubic_cp1_clr, 0.5));
				}
				
				if(control_point_size > 0)
				{
					c.draw_rectangle(
						p0.x - cps, p0.y - cps,
						p0.x + cps, p0.y + cps,
						45, multiply_alpha(cubic_cp1_clr, 0.5));
					c.draw_rectangle(
						p3.x - cps, p3.y - cps,
						p3.x + cps, p3.y + cps,
						45, multiply_alpha(cubic_cp1_clr, 0.5));
				}
			}
		}
	}
	
	/** Draws the skeleton, ie. lines connecting each vertex. */
	void draw_outline(canvas@ c, MultiCurve@ curve, const float zoom_factor=1)
	{
		if(outline_width <= 0 || curve.vertex_count <= 0 || curve.type == CurveType::Linear)
			return;
		
		const float ow = outline_width * zoom_factor;
		
		if(!check_clip(curve, ow))
			return;
		
		CurveVertex@ p1 = curve.vertices[0];
		
		for(int i = 1, end = curve.vertex_count + (curve.closed ? 1 : 0); i < end; i++)
		{
			CurveVertex@ p2 = curve.vert(i);
			
			if(clip)
			{
				const float x1 = p1.x < p2.x ? p1.x : p2.x;
				const float y1 = p1.y < p2.y ? p1.y : p2.y;
				const float x2 = p1.x > p2.x ? p1.x : p2.x;
				const float y2 = p1.y > p2.y ? p1.y : p2.y;
				if(x1 > _clip_x2 || x2 < _clip_x1 || y1 > _clip_y2 || y2 < _clip_y1)
				{
					@p1 = p2;
					continue;
				}
			}
			
			c.draw_line(p1.x, p1.y, p2.x, p2.y, ow, outline_clr);
			
			@p1 = p2;
		}
	}
	
	/** Draws the curve and normals. */
	void draw_curve(canvas@ c, MultiCurve@ curve, const float zoom_factor=1)
	{
		if(!check_clip(curve, max(line_width, max(normal_length, normal_width)), zoom_factor))
			return;
		
		const bool draw_curve = line_width > 0;
		const bool draw_normal = normal_width > 0 && normal_length > 0;
		
		if(curve_segments <= 0 || !draw_curve && draw_normal)
			return;
		
		const int v_count = curve.closed ? curve.vertex_count - 1 : curve.vertex_count - 2;
		const int count = curve.type != CurveType::Linear ? curve_segments : 1;
		const float adaptive_angle = adaptive_max_subdivisions > 0 ? this.adaptive_angle * DEG2RAD : 0;
		const bool eval_normal = draw_curve && draw_normal || draw_normal || adaptive_angle > 0;
		const int subdivisions = curve.type != CurveType::Linear && adaptive_angle > 0 ? adaptive_max_subdivisions : 0;
		
		for(int i = 0; i <= v_count; i++)
		{
			if(clip)
			{
				CurveVertex@ v = curve.vertices[i];
				
				if(clip && (v.x1 > _clip_x2 || v.x2 < _clip_x1 || v.y1 > _clip_y2 || v.y2 < _clip_y1))
					continue;
			}
			
			float t1 = 0;
			float x1 = 0;
			float y1 = 0;
			float n1x = 0;
			float n1y = 0;
			
			for(int j = 0; j <= count; j++)
			{
				const float t2 = float(j) / count;
				
				float x2, y2, n2x, n2y;
				
				draw_segment(
					c, curve, zoom_factor,
					i, v_count,
					t1, t2, t2, x1, y1, n1x, n1y,
					j > 0, draw_curve, draw_normal, eval_normal,
					adaptive_angle, subdivisions,
					x2, y2, n2x, n2y);
				
				t1 = t2;
				x1 = x2;
				y1 = y2;
				n1x = n2x;
				n1y = n2y;
			}
		}
	}
	
	private void draw_segment(
		canvas@ c, MultiCurve@ curve, const float zoom_factor,
		const int segment_index, const int segment_max,
		const float t1, const float t2, const float final_t,
		const float x1, const float y1, const float n1x, const float n1y,
		const bool do_draw, const bool draw_curve, const bool draw_normal, const bool eval_normal,
		const float adaptive_angle, const int sub_divisions,
		float &out x2, float &out y2, float &out n2x, float &out n2y,
		float ix2=0, float iy2=0, float in2x=0, float in2y=0)
	{
		if(in2x == 0 && in2y == 0)
		{
			if(eval_normal)
			{
				curve.eval(segment_index, t2, x2, y2, n2x, n2y);
			}
			else
			{
				curve.eval_point(segment_index, t2, x2, y2);
			}
		}
		else
		{
			x2 = ix2;
			y2 = iy2;
			n2x = in2x;
			n2y = in2y;
		}
		
		if(do_draw && sub_divisions > 0)
		{
			bool subdivide = true;
			
			if(adaptive_min_length > 0)
			{
				const float dx = x2 - x1;
				const float dy = y2 - y1;
				if(dx * dx + dy * dy <= adaptive_min_length * adaptive_min_length)
				{
					subdivide = false;
				}
			}
			
			if(subdivide && acos(clamp(n1x * n2x + n1y * n2y, -1.0, 1.0)) > adaptive_angle)
			{
				const float tm = (t1 + t2) * 0.5;
				float mx, my;
				float nmx, nmy;
				
				// Left
				draw_segment(
					c, curve, zoom_factor,
					segment_index, segment_max,
					t1, tm, final_t, x1, y1, n1x, n1y,
					true, draw_curve, true, true,
					adaptive_angle, sub_divisions - 1,
					mx, my, nmx, nmy,
					0, 0, 0, 0);
				
				// Right
				draw_segment(
					c, curve, zoom_factor,
					segment_index, segment_max,
					tm, t2, final_t, mx, my, nmx, nmy,
					true, draw_curve, true, true,
					adaptive_angle, sub_divisions - 1,
					mx, my, nmx, nmy,
					x2, y2, n2x, n2y);
				
				return;
			}
		}
		
		if(draw_normal)
		{
			const float l = normal_length * (t2 == final_t ? 1.0 : normal_multiplier_adaptive) * zoom_factor;
			c.draw_line(
				x2, y2, x2 + n2x * l, y2 + n2y * l, normal_width * zoom_factor,
				t2 == final_t && normal_adaptive_clr != 0 ? normal_clr : normal_adaptive_clr);
		}
		
		if(do_draw && draw_curve)
		{
			const uint clr = @segment_colour_callback != null
				? segment_colour_callback.get_curve_line_colour(curve, segment_index, segment_max, t2)
				: line_clr;
			c.draw_line(x1, y1, x2, y2, line_width * zoom_factor, clr);
		}
	}
	
	/** Draws the actual curve. */
	void draw_vertices(canvas@ c, MultiCurve@ curve, const float zoom_factor=1)
	{
		if(vertex_size <= 0)
			return;
		
		const float vs = vertex_size * zoom_factor;
		const float vs2 = sqrt(vs * vs + vs * vs);
		
		if(!check_clip(curve, vs))
			return;
		
		for(int i = 0; i < curve.vertex_count; i++)
		{
			if(i == hovered_vertex_index && hovered_control_point_index == 0)
				continue;
			
			CurveVertex@ p = curve.vertices[i];
			
			if(clip && (p.x > _clip_x2 || p.x < _clip_x1 || p.y > _clip_y2 || p.y < _clip_y1))
				continue;
			
			c.draw_rectangle(
				p.x - vs, p.y - vs,
				p.x + vs, p.y + vs,
				45, vertex_clr);
		}
		
		if(hovered_vertex_index >= 0 && hovered_control_point_index == 0 && hovered_vertex_index < curve.vertex_count)
		{
			CurveVertex@ p = curve.vertices[hovered_vertex_index];
			
			const float hvs = vs * hover_vertex_size_mult;
			
			if(hover_outline_thickness > 0 && hover_outline_clr != 0)
			{
				const float ovs = hvs + hover_outline_thickness * zoom_factor;
				c.draw_rectangle(
					p.x - ovs, p.y - ovs,
					p.x + ovs, p.y + ovs,
					45, hover_outline_clr);
			}
			
			c.draw_rectangle(
				p.x - hvs, p.y - hvs,
				p.x + hvs, p.y + hvs,
				45, vertex_clr);
		}
		
		if(hovered_control_point_index != 0 && control_point_size > 0)
		{
			if(curve.type == CurveType::QuadraticBezier)
			{
				CurveVertex@ p = curve.vertices[hovered_vertex_index];
				
			}
		}
	}
	
	void draw_hovered(canvas@ c, MultiCurve@ curve, const float zoom_factor=1)
	{
		if(hovered_control_point_index != 0 && control_point_size > 0)
		{
			const float cps = control_point_size * hover_control_point_size_mult * zoom_factor;
			
			if(curve.type == CurveType::QuadraticBezier)
			{
				CurveVertex@ p = curve.vertices[hovered_vertex_index];
				CurveControlPoint@ cp = p.quad_control_point;
				
				if(hover_outline_thickness > 0 && hover_outline_clr != 0)
				{
					const float ovs = cps + hover_outline_thickness * zoom_factor;
					
					c.draw_rectangle(
						p.x + cp.x - ovs, p.y + cp.y - ovs,
						p.x + cp.x + ovs, p.y + cp.y + ovs,
						45, colour::lerp(quad_cp_clr, hover_outline_clr, hover_outline_blend));
				}
				
				c.draw_rectangle(
					p.x + cp.x - cps, p.y + cp.y - cps,
					p.x + cp.x + cps, p.y + cp.y + cps,
					45, quad_cp_clr);
			}
			else if(curve.type == CurveType::CubicBezier)
			{
				CurveVertex@ p = curve.vertices[hovered_control_point_index == 1 ? hovered_vertex_index : (hovered_vertex_index + 1) % curve.vertex_count];
				CurveControlPoint@ cp = hovered_control_point_index == 1 ? p.cubic_control_point_2 : p.cubic_control_point_1;
				const uint cp_clr = hovered_control_point_index == 1 ? cubic_cp2_clr : cubic_cp1_clr;
				const float x = p.x + cp.x;
				const float y = p.y + cp.y;
				
				if(hover_outline_thickness > 0 && hover_outline_clr != 0)
				{
					const float ovs = cps + hover_outline_thickness * zoom_factor;
					
					c.draw_rectangle(
						x - ovs, y - ovs,
						x + ovs, y + ovs,
						45, colour::lerp(cp_clr, hover_outline_clr, hover_outline_blend));
				}
				
				c.draw_rectangle(
					x - cps, y - cps,
					x + cps, y + cps,
					45, cp_clr);
			}
			else if(
				curve.type == CatmullRom && !curve.closed && curve.end_controls == CurveEndControl::Manual &&
				(hovered_control_point_index == -1 || hovered_control_point_index == curve.vertex_count))
			{
				CurveControlPoint@ cp = hovered_control_point_index == -1 ? curve.control_point_start : curve.control_point_end;
				CurveVertex@ v = hovered_control_point_index == -1 ? curve.first_vertex : curve.last_vertex;
				
				if(hover_outline_thickness > 0 && hover_outline_clr != 0)
				{
					const float ovs = cps + hover_outline_thickness * zoom_factor;
					
					c.draw_rectangle(
						v.x + cp.x - ovs, v.y + cp.y - ovs,
						v.x + cp.x + ovs, v.y + cp.y + ovs,
						45, colour::lerp(cubic_cp1_clr, hover_outline_clr, hover_outline_blend));
				}
				
				c.draw_rectangle(
					v.x + cp.x - cps, v.y + cp.y - cps,
					v.x + cp.x + cps, v.y + cp.y + cps,
					45, cubic_cp1_clr);
			}
		}
		else if(hovered_vertex_index >= 0 && hovered_vertex_index < curve.vertex_count)
		{
			CurveVertex@ p = curve.vertices[hovered_vertex_index];
			
			const float hvs = vertex_size * zoom_factor * hover_vertex_size_mult;
			
			if(hover_outline_thickness > 0 && hover_outline_clr != 0)
			{
				const float ovs = hvs + hover_outline_thickness * zoom_factor;
				c.draw_rectangle(
					p.x - ovs, p.y - ovs,
					p.x + ovs, p.y + ovs,
					45, colour::lerp(vertex_clr, hover_outline_clr, hover_outline_blend));
			}
			
			c.draw_rectangle(
				p.x - hvs, p.y - hvs,
				p.x + hvs, p.y + hvs,
				45, vertex_clr);
		}
	}
	
	/** Draws the bounding box of the curve. */
	void draw_bounding_box(canvas@ c, MultiCurve@ curve, const float zoom_factor=1, const float segment_padding=0)
	{
		if(!check_clip(curve, max(segment_bounding_box_width, bounding_box_width), zoom_factor))
			return;
		
		if(segment_bounding_box_width > 0)
		{
			const float w = segment_bounding_box_width * zoom_factor;
			const float p = segment_padding * zoom_factor;
			
			const int end = curve.closed ? curve.vertex_count : curve.vertex_count - 1;
			for(int i = 0; i < end; i++)
			{
				CurveVertex@ v = curve.vertices[i];
				
				if(clip && (v.x1 > _clip_x2 || v.x2 < _clip_x1 || v.y1 > _clip_y2 || v.y2 < _clip_y1))
					continue;
				
				// Left
				c.draw_rectangle(
					v.x1 - p - w, v.y1 - p,
					v.x1 - p, v.y2 + p,
					0, segment_bounding_box_clr);
				// Right
				c.draw_rectangle(
					v.x2 + p, v.y1 - p,
					v.x2 + p + w, v.y2 + p,
					0, segment_bounding_box_clr);
				// Top
				c.draw_rectangle(
					v.x1 - p - w, v.y1 - p - w,
					v.x2 + p + w, v.y1 - p,
					0, segment_bounding_box_clr);
				// Bottom
				c.draw_rectangle(
					v.x1 - p - w, v.y2 + p,
					v.x2 + p + w, v.y2 + p + w,
					0, segment_bounding_box_clr);
			}
		}
		
		if(bounding_box_width <= 0)
			return;
		
		const float w = bounding_box_width * zoom_factor;
		
		// Left
		c.draw_rectangle(
			curve.x1 - w, curve.y1,
			curve.x1, curve.y2,
			0, bounding_box_clr);
		// Right
		c.draw_rectangle(
			curve.x2, curve.y1,
			curve.x2 + w, curve.y2,
			0, bounding_box_clr);
		// Top
		c.draw_rectangle(
			curve.x1 - w, curve.y1 - w,
			curve.x2 + w, curve.y1,
			0, bounding_box_clr);
		// Bottom
		c.draw_rectangle(
			curve.x1 - w, curve.y2,
			curve.x2 + w, curve.y2 + w,
			0, bounding_box_clr);
	}
	
	/** Draws the pre-calculated sub divisions of the curve. */
	void draw_arch_lengths(canvas@ c, MultiCurve@ curve, const float zoom_factor=1)
	{
		const float lw = line_width * zoom_factor;
		const float nl = normal_length * zoom_factor * 0.5;
		
		if(!check_clip(curve, max(line_width, nl)))
			return;
		
		const int v_count = curve.closed ? curve.vertex_count - 1 : curve.vertex_count - 2;
		const bool draw_normal = normal_width > 0 && normal_length > 0;
		
		for(int i = 0; i <= v_count; i++)
		{
			CurveVertex@ v = curve.vertices[i];
			array<CurveArc>@ arcs = @v.arcs;
			int arc_count = v.arc_count;
			
			if(arc_count <= 0)
				continue;
			
			if(clip && (v.x1 > _clip_x2 || v.x2 < _clip_x1 || v.y1 > _clip_y2 || v.y2 < _clip_y1))
				continue;
			
			float x1 = arcs[0].x;
			float y1 = arcs[0].y;
			for(int j = 1; j < arc_count; j++)
			{
				CurveArc@ arc = @arcs[j];
				
				const uint clr = @segment_colour_callback != null
					? segment_colour_callback.get_curve_line_colour(curve, i, v_count, arc.t)
					: line_clr;
				c.draw_line(x1, y1, arc.x, arc.y, lw, clr);
				
				x1 = arc.x;
				y1 = arc.y;
			}
			
			if(draw_normal)
			{
				for(int j = 1; j < arc_count; j++)
				{
					CurveArc@ arc = @arcs[j];
					
					const uint clr = @segment_colour_callback != null
						? segment_colour_callback.get_curve_line_colour(curve, i, v_count, arc.t)
						: line_clr;
					c.draw_line(
						arc.x - arc.nx * nl, arc.y - arc.ny * nl, arc.x + arc.nx * nl, arc.y + arc.ny * nl, normal_width * zoom_factor,
						clr);
				}
			}
		}
	}
	
	void draw_segment_labels(canvas@ c, MultiCurve@ curve, const float zoom_factor=1)
	{
		if(segment_index_label_clr == 0 || segment_index_label_scale == 0)
			return;
		
		tf.colour(segment_index_label_clr);
		
		const int v_count = curve.closed ? curve.vertex_count - 1 : curve.vertex_count - 2;
		const float o = segment_index_label_offset * zoom_factor;
		const float scale = segment_index_label_scale * zoom_factor;
		
		for(int i = 0; i <= v_count; i++)
		{
			float x, y, nx, ny;
			curve.eval(i, 0.5, x, y, nx, ny);
			
			tf.text(i + '');
			
			if(abs(nx) > abs(ny))
			{
				tf.align_horizontal(int(-sign(nx)));
				tf.align_vertical(0);
			}
			else
			{
				tf.align_horizontal(0);
				tf.align_vertical(int(-sign(ny)));
			}
			
			c.draw_text(tf, x + nx * o, y + ny * o, scale, scale, 0);
		}
	}
	
	private bool check_clip(MultiCurve@ curve, const float padding, const float zoom_factor=1, const bool negative_result=false)
	{
		if(!clip)
		{
			_clip_x1 = _clip_y1 = _clip_x2 = _clip_y2 = 0;
			return true;
		}
		
		_clip_x1 = clip_x1 - padding * zoom_factor;
		_clip_y1 = clip_y1 - padding * zoom_factor;
		_clip_x2 = clip_x2 + padding * zoom_factor;
		_clip_y2 = clip_y2 + padding * zoom_factor;
		
		return curve.x1 <= _clip_x2 && curve.x2 >= _clip_x1 && curve.y1 <= _clip_y2 && curve.y2 >= _clip_y1
			? true : negative_result;
	}
	
}

interface MultiCurveDebugColourCallback
{
	
	uint get_curve_line_colour(const MultiCurve@ curve, const int segment_index, const int segment_max, const float t);
	
}
