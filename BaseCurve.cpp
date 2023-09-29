class BaseCurve
{
	
	// TODO: control_point_start/end should always be moved relative to the start/end vertices
	//       when `end_controls` is not `Manual`.
	// TODO: Move debug drawing methods into here.
	// TODO: curve x/y, scale x/y, and rotation.
	
	[option,Linear,QuadraticBezier,CubicBezier,CatmullRom,BSpline]
	private CurveType _type = CubicBezier;
	
	/// Do not set directly.
	[persist] int vertex_count;
	
	/// Do not set directly.
	[persist] array<CurveVertex> vertices;
	
	/// The control points for the quadratic bezier. Will always have `vertex_count` length
	/// and each item represents the control points to the "right" of corresponding vertex.
	/// Do not set directly.
	[persist] array<CurveVertex> quadratic_bezier_control_points;
	
	/// The control points for the cubic bezier. Will always have `vertex_count * 2` length
	/// and each pair represents the corresponding vertex left and right control points.
	/// Do not set directly.
	[persist] array<CurveVertex> cubic_bezier_control_points;
	
	[persist] CurveEndControl _end_controls = AutomaticAngle;
	
	/// Only applicable for CatmullRom splines with `end_controls` set to `Manual`.
	[persist] CurveVertex control_point_start;
	
	/// Only applicable for CatmullRom splines with `end_controls` set to `Manual`.
	[persist] CurveVertex control_point_end;
	
	/// Controls the global tension for CatmullRom splines.
	[persist] float tension = 1;
	
	/// If true the start and end points of this path will automatically connect to each other.
	[persist] bool closed;
	
	/// Temp points used when calculating automatic end control points.
	private CurveVertex p0;
	private CurveVertex p3;
	
	BaseCurve()
	{
		control_point_start.type = None;
		control_point_end.type = None;
	}
	
	CurveEndControl end_controls
	{
		get const { return _end_controls; }
		set
		{
			if(value == _end_controls)
				return;
			
			_end_controls = value;
			
			if(_end_controls == Manual)
			{
				check_control_point_start();
				check_control_point_end();
			}
		}
	}
	
	CurveType type
	{
		get const { return _type; }
		set
		{
			if(value == _type)
				return;
			
			_type = value;
			
			calc_bezier_control_points();
		}
	}
	
	const CurveVertex@ first_vertex
	{
		get { return vertex_count > 0 ? vertices[0] : null; }
	}
	
	const CurveVertex@ last_vertex
	{
		get { return vertex_count > 0 ? vertices[vertex_count - 1] : null; }
	}
	
	void clear()
	{
		vertices.resize(0);
		cubic_bezier_control_points.resize(0);
		quadratic_bezier_control_points.resize(0);
		vertex_count = 0;
		
		control_point_start.type = None;
		control_point_end.type = None;
	}
	
	void add_vertex(const float x, const float y)
	{
		vertices.insertLast(CurveVertex(x, y));
		vertex_count++;
		
		cubic_bezier_control_points.insertLast(CurveVertex(NAN, NAN));
		cubic_bezier_control_points.insertLast(CurveVertex(NAN, NAN));
		quadratic_bezier_control_points.insertLast(CurveVertex(NAN, NAN));
		
		calc_bezier_control_points();
	}
	
	/// 
	void calc_bezier_control_points(const bool force=false, const int from_index=0, const int count=0xffffff)
	{
		switch(_type)
		{
			case CurveType::CubicBezier:
				calc_cubic_bezier_control_points(force, from_index, count);
				break;
			case CurveType::QuadraticBezier:
				calc_quadratic_bezier_control_points(force, from_index, count);
				break;
		}
	}
	
	/// 
	void calc_cubic_bezier_control_points(const bool force=false, const int from_index=0, const int count=0xffffff)
	{
		if(vertex_count == 1 && from_index == 0)
		{
			CurveVertex@ cp1 = @cubic_bezier_control_points[0];
			CurveVertex@ cp2 = @cubic_bezier_control_points[1];
			if(force || is_nan(cp1.x))
			{
				cp1.x = -50;
				cp1.y = -25;
			}
			if(force || is_nan(cp2.x))
			{
				cp2.x = 50;
				cp2.y = 25;
			}
			return;
		}
		
		const int end = from_index + count < vertex_count ? from_index + count : vertex_count;
		for(int i = from_index; i < end; i++)
		{
			const CurveVertex@ p1 = @vertices[i];
			CurveVertex@ cp1 = @cubic_bezier_control_points[i * 2];
			CurveVertex@ cp2 = @cubic_bezier_control_points[i * 2 + 1];
			
			if(!force && !is_nan(cp1.x) && !is_nan(cp2.x))
				continue;
			
			const CurveVertex@ p0 = vert(i, -1);
			const CurveVertex@ p2 = vertex_count > 2 ? vert(i, 1) : vertices[i];
			const float tx = p0.x - p2.x;
			const float ty = p0.y - p2.y;
			
			if(force || is_nan(cp1.x))
			{
				cp1.x = tx * 0.25;
				cp1.y = ty * 0.25;
			}
			
			if(force || is_nan(cp2.x))
			{
				cp2.x = -tx * 0.25;
				cp2.y = -ty * 0.25;
			}
		}
	}
	
	/// 
	void calc_quadratic_bezier_control_points(const bool force=false, const int from_index=0, const int count=0xffffff)
	{
		const int end = from_index + count < vertex_count ? from_index + count : vertex_count;
		for(int i = from_index; i < end; i++)
		{
			const CurveVertex@ p1 = @vertices[i];
			CurveVertex@ cp = @quadratic_bezier_control_points[i];
			
			if(!force && !is_nan(cp.x))
				continue;
			
			const CurveVertex@ p0 = vert(i, -1);
			const CurveVertex@ p2 = vertex_count > 2 ? vert(i, 1) : vertices[i];
			const float tx = p2.x - p0.x;
			const float ty = p2.y - p0.y;
			
			if(force || is_nan(cp.x))
			{
				cp.x = tx * 0.5;
				cp.y = ty * 0.5;
			}
		}
	}
	
	CurveVertex@ vert(const int i, const int offset=0)
	{
		return vertices[((i + offset) % vertex_count + vertex_count) % vertex_count];
	}
	
	void calc(const float t, float &out x, float &out y, float &out normal_x, float &out normal_y)
	{
		if(vertex_count == 0)
		{
			x = 0;
			y = 0;
			normal_x = 1;
			normal_y = 0;
			return;
		}
		if(vertex_count == 1)
		{
			const CurveVertex@ p0 = @vertices[0];
			x = p0.x;
			y = p0.y;
			normal_x = 1;
			normal_y = 0;
			return;
		}
		
		switch(_type)
		{
			case Linear:
				calc_linear(t, x, y, normal_x, normal_y);
				break;
			case QuadraticBezier:
				calc_quadratic_bezier(t, x, y, normal_x, normal_y);
				break;
			case CubicBezier:
				calc_cubic_bezier(t, x, y, normal_x, normal_y);
				break;
			case CatmullRom:
				calc_catmull_rom(t, x, y, normal_x, normal_y);
				break;
			case BSpline:
				calc_b_spline(t, x, y, normal_x, normal_y);
				break;
			default:
				x = 0;
				y = 0;
				normal_x = 1;
				normal_y = 0;
				break;
		}
	}
	
	void calc_linear(const float t, float &out x, float &out y, float &out normal_x, float &out normal_y)
	{
		int i;
		float ti;
		calc_segment_t(t, ti, i);
		
		// Get vertices.
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p2 = vert(i, 1);
		
		// Calculate the point.
		const float dx = p2.x - p1.x;
		const float dy = p2.y - p1.y;
		x = p1.x + dx * ti;
		y = p1.y + dy * ti;
		
		// Calculate the normal vector.
		const float length = sqrt(dx * dx + dy * dy);
		normal_x = dy / length;
		normal_y = -dx / length;
	}
	
	void calc_catmull_rom(const float t, float &out x, float &out y, float &out normal_x, float &out normal_y)
	{
		if(vertex_count == 2)
		{
			const CurveVertex@ p0 = @vertices[0];
			const CurveVertex@ p1 = @vertices[1];
			const float dx = p1.x - p0.x;
			const float dy = p1.y - p0.y;
			x = p0.x + dx * t;
			y = p0.y + dy * t;
			const float length = sqrt(dx * dx + dy * dy);
			normal_x = dy / length;
			normal_y = -dx / length;
			return;
		}
		
		int i;
		float ti;
		calc_segment_t(t, ti, i);
		
		// Get vertices.
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p2 = vert(i, 1);
		
		// Get control points.
		const CurveVertex@ p0 = p1.type != Square
			? closed || i > 0
				? vert(i, -1)
				: _end_controls != Manual
					? this.p0.extrapolate(p1, p2,
						_end_controls == CurveEndControl::AutomaticAngle && vertex_count >= 3 ? @vertices[2] : null)
					: check_control_point_start()
			: p1;
		const CurveVertex@ p3 = p2.type != Square
			? closed || i < vertex_count - 2
				? vert(i, 2)
				: _end_controls != Manual
					? this.p3.extrapolate(p2, p1,
						_end_controls == CurveEndControl::AutomaticAngle && vertex_count >= 3 ? @vertices[vertex_count - 3] : null)
					: check_control_point_end()
			: p2;
		
		// Calculate the point.
		const float t2 = ti * ti;
		const float t3 = t2 * ti;
		
		const float st = (tension * p1.tension) * 2;
		
		const float dv1x = (p2.x - p0.x) / st;
		const float dv1y = (p2.y - p0.y) / st;
		const float dv2x = (p3.x - p1.x) / st;
		const float dv2y = (p3.y - p1.y) / st;
		
		const float c0 = 2 * t3 - 3 * t2 + 1;
		const float c1 = t3 - 2 * t2 + ti;
		const float c2 = -2 * t3 + 3 * t2;
		const float c3 = t3 - t2;
		x = c0 * p1.x + c1 * dv1x + c2 * p2.x + c3 * dv2x;
		y = c0 * p1.y + c1 * dv1y + c2 * p2.y + c3 * dv2y;
		
		// Calculate the normal.
		normal_x =
			((3 * t2 - 4 * ti + 1) * (p2.y - p0.y)) / st +
			((3 * t2 - 2 * ti) * (p3.y - p1.y)) / st +
			p1.y * (6 * t2 - 6 * ti) + p2.y * (6 * ti - 6 * t2);
		normal_y = -(
		((3 * t2 - 4 * ti + 1) * (p2.x - p0.x)) / st +
			((3 * t2 - 2 * ti) * (p3.x - p1.x)) / st +
			p1.x * (6 * t2 - 6 * ti) + p2.x * (6 * ti - 6 * t2));
		
		const float length = sqrt(normal_x * normal_x + normal_y * normal_y);
		normal_x /= length;
		normal_y /= length;
	}
	
	void calc_quadratic_bezier(const float t, float &out x, float &out y, float &out normal_x, float &out normal_y)
	{
		int i;
		float ti;
		calc_segment_t(t, ti, i);
		
		// Get vertices.
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p2 = vert(i, 1);
		
		// Get control points.
		const CurveVertex@ cp = quadratic_bezier_control_points[i];
		const float cpx = p1.x + cp.x;
		const float cpy = p1.y + cp.y;
		
		// Calculate the point.
		const float u = 1 - ti;
		const float tt = ti * ti;
		const float uu = u * u;
		const float ut2 = 2 * u * ti;
		
		
		// Normal
		if(p1.weight == 1)
		{
			x = uu * p1.x + ut2 * cpx + tt * p2.x;
			y = uu * p1.y + ut2 * cpy + tt * p2.y;
			
			normal_x =  2 * (u * (cpy - p1.y) + ti * (p2.y - cpy));
			normal_y = -2 * (u * (cpx - p1.x) + ti * (p2.x - cpx));
		}
		// Conic
		else
		{
			const float w = p1.weight;
			const float denominator = uu + ut2 * w + tt;
			x = (uu * p1.x + ut2 * w * cpx + tt * p2.x) / denominator;
			y = (uu * p1.y + ut2 * w * cpy + tt * p2.y) / denominator;
			
			const float prime = 2 * u * w - 2 * ti * w - 2 * u + 2 * ti;
			normal_x =
				(-2 * p1.y * u + 2 * cpy * u * w - 2 * cpy * ti * w + 2 * p2.y * ti) / denominator -
				(prime * (p1.y * uu + cpy * ut2 * w + p2.y * tt)) / (denominator * denominator);
			normal_y =
				-((-2 * p1.x * u + 2 * cpx * u * w - 2 * cpx * ti * w + 2 * p2.x * ti) / denominator -
				(prime * (p1.x * uu + cpx * ut2 * w + p2.x * tt)) / (denominator * denominator));
		}
		
		const float length = sqrt(normal_x * normal_x + normal_y * normal_y);
		normal_x /= length;
		normal_y /= length;
	}
	
	void calc_cubic_bezier(const float t, float &out x, float &out y, float &out normal_x, float &out normal_y)
	{
		int i;
		float ti;
		calc_segment_t(t, ti, i);
		int i2 = i * 2;
		
		// Get vertices.
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p2 = vert(i, 1);
		
		// Get control points.
		const CurveVertex@ cp1 = p1.type != Square
			? cubic_bezier_control_points[i2 + 1]
			: p1;
		const CurveVertex@ cp2 = p2.type != Square
			? cubic_bezier_control_points[mod(i + 1, vertex_count) * 2]
			: p2;
		const float cp1x = p1.x + cp1.x;
		const float cp1y = p1.y + cp1.y;
		const float cp2x = p2.x + cp2.x;
		const float cp2y = p2.y + cp2.y;
		
		// Calculate the point.
		const float u = 1 - ti;
		const float tt = ti * ti;
		const float tt3 = tt * ti;
		const float uu = u * u;
		const float uuu = uu * u;
		
		// Normal.
		if(p1.weight == 1 && p2.weight == 1 && cp1.weight == 1 && cp2.weight == 1)
		{
			x = uuu * p1.x + 3 * uu * ti * cp1x + 3 * u * tt * cp2x + tt3 * p2.x;
			y = uuu * p1.y + 3 * uu * ti * cp1y + 3 * u * tt * cp2y + tt3 * p2.y;
			// Calculate the normal vector.
			normal_x = -3 * p1.y * uu + 3 * cp1y * (3 * uu - 2 * u) + 3 * cp2y * (2 * ti - 3 * tt) + 3 * p2.y * tt;
			normal_y = -(-3 * p1.x * uu + 3 * cp1x * (3 * uu - 2 * u) + 3 * cp2x * (2 * ti - 3 * tt) + 3 * p2.x * tt);
		}
		// Weighted
		else
		{
			const float f0 = uuu * p1.weight;
			const float f1 = 3 * uu * ti * cp1.weight;
			const float f2 = 3 * u * tt * cp2.weight;
			const float f3 = tt3 * p2.weight;
			const float basis = f0 + f1 + f2 + f3;
			if(f0 + f1 + f2 + f3 == 0)
			 puts(f0 + f1 + f2 + f3);
			x = (f0 * p1.x + f1 * cp1x + f2 * cp2x + f3 * p2.x) / basis;
			y = (f0 * p1.y + f1 * cp1y + f2 * cp2y + f3 * p2.y) / basis;
			
			normal_x =
				(
					-3 * p1.y * p1.weight * uu + 3 * cp1y * cp1.weight * uu - 6 * cp1y * cp1.weight * ti * u -
					3 * cp2y * cp2.weight * tt + 6 * cp2y * cp2.weight * ti * u + 3 * p2.y * p2.weight * tt
				) / basis
				- (
					(-3 * p1.weight * uu + 3 * cp1.weight * uu - 6 * cp1.weight * ti * u - 3 * cp2.weight * tt + 6 * cp2.weight * ti * u + 3 * p2.weight * tt) *
					(p1.y * p1.weight * uuu + 3 * cp1y * cp1.weight * ti * uu + 3 * cp2y * cp2.weight * tt * u + p2.y * p2.weight * tt3))
				/ (basis * basis);
			normal_y = -(
				(
					-3 * p1.x * p1.weight * uu + 3 * cp1x * cp1.weight * uu - 6 * cp1x * cp1.weight * ti * u -
					3 * cp2x * cp2.weight * tt + 6 * cp2x * cp2.weight * ti * u + 3 * p2.x * p2.weight * tt
				) / basis
				- (
					(-3 * p1.weight * uu + 3 * cp1.weight * uu - 6 * cp1.weight * ti * u - 3 * cp2.weight * tt + 6 * cp2.weight * ti * u + 3 * p2.weight * tt) *
					(p1.x * p1.weight * uuu + 3 * cp1x * cp1.weight * ti * uu + 3 * cp2x * cp2.weight * tt * u + p2.x * p2.weight * tt3)
				) / (basis * basis));
		}
		
		const float length = sqrt(normal_x * normal_x + normal_y * normal_y);
		normal_x /= length;
		normal_y /= length;
	}
	
	/// The smoothness of the b-spline. Must be > 1 and < `vertex_count`.
	[persist] int b_spline_degree = 2;
	
	private array<float> knots;
	private array<CurveVertex> v;
	
	/// https://github.com/thibauts/b-spline
	void calc_b_spline(const float t, float &out x, float &out y, float &out normal_x, float &out normal_y)
	{
		const int degree = clamp(b_spline_degree, 2, vertex_count - 1);
		
		// Generate knots.
		const int knots_length = vertex_count + degree + 1;
		if(int(knots.length) < knots_length)
		{
			knots.resize(knots_length);
		}
		for(int i = 0; i < knots_length; i++)
		{
			// A clamped b-spline touches the first and last vertices.
			// To do this make sure the first and last knot are repeated `degree + 1` times.
			knots[i] = min(max(i - degree, 0), knots_length - (degree) * 2 - 1);
		}
		
		const int domain0 = degree;
		const int domain1 = knots_length - 1 - degree;
		
		// Remap t to the domain where the spline is defined
		const float low  = knots[domain0];
		const float high = knots[domain1];
		const float ti = clamp01(t) * (high - low) + low;
		
		// Find s (the spline segment) for the t value provided.
		int s = 0;
		for(s = domain0; s < domain1; s++)
		{
			if(ti >= knots[s] && ti <= knots[s + 1])
				break;
		}
		
		// Convert points to homogeneous coordinates
		if(int(v.length) < vertex_count)
		{
			v.resize(vertex_count);
		}
		
		for(int i = 0; i < vertex_count; i++)
		{
			CurveVertex@ vp = @v[i];
			CurveVertex@ p = @vertices[i];
			vp.x = p.x * p.weight;
			vp.y = p.y * p.weight;
			vp.weight = p.weight;
		}
		
		// Level (l) goes from 1 to the curve degree + 1.
		for(int l = 1; l <= degree + 1; l++)
		{
			// Build level l of the pyramid
			for(int i = s; i > s - degree - 1 + l; i--)
			{
				const float alpha = (ti - knots[i]) / (knots[i + degree + 1 - l] - knots[i]);
				
				// Interpolate each component
				CurveVertex@ vpp = @v[i - 1];
				CurveVertex@ vp = @v[i];
				vp.x = (1 - alpha) * vpp.x + alpha * vp.x;
				vp.y = (1 - alpha) * vpp.y + alpha * vp.y;
			}
		}
		//for(int j = 1; j <= degree; j++)
		//{
		//	left[j] = u - knots[span + 1 - j];
		//	right[j] = knots[span + j] - u;
		//	saved = 0.0;
		//	
		//	for(int r = 0; r < j; r++)
		//	{
		//		const float temp = n[r] / (right[r + 1] + left[j - r]);
		//		n[r] = saved + right[r + 1] * temp;
		//		saved = left[j - r] * temp;
		//	}
		//	n[j] = saved;
		//}
		
		// Convert back to cartesian and return.
		CurveVertex@ vp = @v[s];
		x = vp.x / vp.weight;
		y = vp.y / vp.weight;
		
		//if(vertex_count <= degree)
		//{
		//	calc_linear(t, x, y, normal_x, normal_y);
		//	return;
		//}
		//
		//int i;
		//float ti;
		//calc_segment_t(t, ti, i);
		//int i2 = i * 2;
		//
		//const CurveVertex@ p1 = closed || i > 1 ? vert(i, -1) : vert(i, -1);
		//const CurveVertex@ p2 = vert(i, 0);
		//const CurveVertex@ p3 = vert(i, 1);
		//const CurveVertex@ p4 = vert(i, 2);
		//
		//const float t2 = ti * ti;
		//const float t3 = t2 * ti;
		//
		//x =
		//	(-t3/6 + t2/2 - ti/2 + 1/6.0) * p1.x +
		//	(t3/2 - t2 + 2/3.0) * p2.x +
		//	(-t3/2 + t2/2 + ti/2 + 1/6.0) * p3.x +
		//	(t3/6) * p4.x;
		//y =
		//	(-t3/6 + t2/2 - ti/2 + 1/6.0) * p1.y +
		//	(t3/2 - t2 + 2/3.0) * p2.y +
		//	(-t3/2 + t2/2 + ti/2 + 1/6.0) * p3.y +
		//	(t3/6) * p4.y;
		
	}
	
	CurveVertex@ get_auto_control_start(CurveVertex@ p_out, const CurveEndControl type)
	{
		if(vertex_count == 0)
			return p_out;
		if(vertex_count == 1)
			return p_out.copy_from(vertices[0]);
		
		return p_out.extrapolate(
			vertices[0], vertices[1],
			type == CurveEndControl::AutomaticAngle && vertex_count >= 3 ? @vertices[2] : null);
	}
	
	CurveVertex@ get_auto_control_end(CurveVertex@ p_out, const CurveEndControl type)
	{
		if(vertex_count == 0)
			return p_out;
		if(vertex_count == 1)
			return p_out.copy_from(vertices[0]);
		
		return p_out.extrapolate(
			vertices[vertex_count - 1], vertices[vertex_count - 2],
			type == CurveEndControl::AutomaticAngle && vertex_count >= 3 ? @vertices[vertex_count - 3] : null);
	}
	
	private CurveVertex@ check_control_point_start()
	{
		if(control_point_start.type != None)
			return control_point_start;
		
		control_point_start.type = Square;
		return get_auto_control_start(control_point_start, CurveEndControl::AutomaticAngle);
	}
	
	private CurveVertex@ check_control_point_end()
	{
		if(control_point_end.type != None)
			return control_point_end;
		
		control_point_end.type = Square;
		return get_auto_control_end(control_point_end, CurveEndControl::AutomaticAngle);
	}
	
	private void calc_segment_t(const float t, float & out ti, int &out i)
	{
		const int max_i = closed ? vertex_count : vertex_count - 1;
		const float tt = t * max_i;
		i = int(tt);
		ti = i < max_i ? tt % 1 : 1.0;
		i = i < max_i ? i : max_i - 1;
	}
	
}

enum CurveType
{
	
	Linear,
	QuadraticBezier,
	CubicBezier,
	CatmullRom,
	BSpline,
	
}

/// Controls how the beginning and end control points are calculated for CatmullRom splines.
enum CurveEndControl
{
	
	/// End control points are calculated automatically by extrapolating the two end vertices.
	Automatic,
	
	/// Same as Automatic, but if the path has 3 or more vertices also takes the angle of the third vertex into account.
	AutomaticAngle,
	
	/// Allows manual control/placement of the control points .
	Manual,
	
}

enum CurveVertexType
{
	
	/// This vertex has not been calculated/assigned yet.
	None,
	
	/// Comes to a sharp point with no handles/control points.
	/// Only applicable for cubic and quadratic bezier curves.
	Square,
	
	/// Control points on either side of a vertex can be moved individually.
	/// Only applicable for cubic and quadratic bezier curves.
	Manual,
	
	/// Angles for control point on either side of a vertex are mirrored but the lengths are independent.
	/// Only applicable for cubic bezier curves.
	Smooth,
	
	/// Both the angles and length for control point on either side of a vertex are mirrored.
	/// Only applicable for cubic bezier curves.
	Mirror,
	
}

class Point
{
	
	[persist] float x;
	[persist] float y;
	
	Point() { }
	
	Point(const float x, const float y)
	{
		this.x = x;
		this.y = y;
	}
	
}

class CurveVertex : Point
{
	
	[option,1:Square,Manual,Smooth,Mirror] CurveVertexType type = Smooth;
	/// The weight for either conic quadratic beziers or b-splines for the segment starting at this vertex.
	[persist] float weight = 1;
	/// A per-segment tension for CatmullRom splines/
	[persist] float tension = 1;
	
	/// Has the segment starting with this segment been invalidated/changed, meaning that the arc length
	/// and look up table need to be recalculated.
	bool invalidated = true;
	/// The approximated length of curve the segment starting with this vertex.
	float arch_length;
	/// The lookup table mapping raw t values to real distances/uniform t values along the curve.
	array<float> arc_lengths;
	
	CurveVertex()
	{
	}
	
	CurveVertex(const float x, const float y)
	{
		this.x = x;
		this.y = y;
	}
	
	CurveVertex@ extrapolate(
		const CurveVertex@ p1, const CurveVertex@ p2, const CurveVertex@ p3=null,
		const float angle_multiplier=1, const float length_multiplier=1)
	{
		// Simple just extend the two points.
		if(@p3 == null)
		{
			x = p1.x + (p1.x - p2.x);
			y = p1.y + (p1.y - p2.y);
		}
		/// With a third point also take the angle difference between the 3 points into account to rotate the extended point.
		else
		{
			//puts('  '+angle_multiplier+' ' + length_multiplier);
			const float a1 = atan2(p1.y - p2.y, p1.x - p2.x);
			const float a2 = shortest_angle(atan2(p2.y - p3.y, p2.x - p3.x), atan2(p1.y - p3.y, p1.x - p3.x));
			const float a = a1 + a2 * angle_multiplier;
			const float length = sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y)) * length_multiplier;
			x = p1.x + cos(a) * length;
			y = p1.y + sin(a) * length;
		}
		
		return this;
	}
	
	CurveVertex@ copy_from(const CurveVertex@ p)
	{
		x = p.x;
		y = p.y;
		return this;
	}
	
}
