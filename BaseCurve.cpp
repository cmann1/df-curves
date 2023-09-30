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
	[persist] bool b_spline_clamped = true;
	
	private array<float> knots;
	private array<CurveVertex> v;
	private array<CurveVertex> curve_wders;
	private array<CurveVertex> curve_ders;
	private array<array<float>> ndu;
	private array<array<float>> ders;
	private array<array<float>> b_a;
	private array<float> n;
	private array<Point> v_ders;
	private array<float> w_ders;
	private array<float> left, right;
	private int knots_length;
	
	/// https://github.com/pradeep-pyro/tinynurbs/tree/master
	void calc_b_spline(const float t, float &out x, float &out y, float &out normal_x, float &out normal_y)
	{
		closed = true;
		const int v_count = closed ? vertex_count + 3 : vertex_count;
		const int clamp_val = b_spline_clamped && !closed ? 0 : 1;
		const int degree = clamp(b_spline_degree, 2, v_count - 1);
		const float u = clamp_val + t * (v_count - degree - clamp_val);
		
		// Generate knots.
		knots_length = v_count + degree + 1;// + (closed ? 3 : 0);
		if(int(knots.length) < knots_length)
		{
			knots.resize(knots_length);
		}
		
		for(int i = 0; i < knots_length; i++)
		{
			// A clamped b-spline touches the first and last vertices.
			// To do this make sure the first and last knot are repeated `degree + 1` times.
			knots[i] = min(max(i - degree + clamp_val, 0), knots_length - (degree - clamp_val) * 2 - 1);
		}
		
		// Compute points using homogenous coordinates.
		if(int(v.length) < v_count)
		{
			v.resize(v_count);
		}
		
		for(int i = 0; i < v_count; i++)
		{
			CurveVertex@ vp = @v[i];
			CurveVertex@ p = @vertices[i % vertex_count];
			vp.x = p.x * p.weight;
			vp.y = p.y * p.weight;
			vp.weight = p.weight;
		}
		
		// Find span and corresponding non-zero basis functions
		const int span = b_spline_find_span(degree, u);
		b_spline_basis(degree, span, u);
		
		// Initialize result to 0s
		x = 0;
		y = 0;
		float w = 0;
		
		// Compute point.
		for(int i = 0; i <= degree; i++)
		{
			CurveVertex@ p = v[span - degree + i];
			const float ni = n[i];
			x += p.x * ni;
			y += p.y * ni;
			w += p.weight * ni;
		}
		
		// Convert back to cartesian coordinates.
		x /= w;
		y /= w;
		
		calc_b_spline_tangent(t, normal_x, normal_y);
		const float temp = normal_x;
		normal_x = normal_y;
		normal_y = -temp;
	}
	
	void curve_derivatives(const int num_ders, const int degree, const float u)
	{
		if(int(curve_wders.length) < num_ders + 1)
		{
			curve_wders.resize(num_ders + 1);
		}
		
		// Assign higher order derivatives to zero.
		for(int i = degree + 1; i <= num_ders; i++)
		{
			CurveVertex@ cd = @curve_wders[i];
			cd.x = 0;
			cd.y = 0;
			cd.weight = 0;
		}
		
		// Find the span and corresponding non-zero basis functions & derivatives.
		const int span = b_spline_find_span(degree, u);
		b_spline_der_basis(degree, span, u, num_ders);
		
		// Compute first num_ders derivatives.
		const int du = num_ders < degree ? num_ders : degree;
		for(int i = 0; i <= du; i++)
		{
			CurveVertex@ cd = @curve_wders[i];
			cd.x = 0;
			cd.y = 0;
			cd.weight = 0;
			
			for (int j = 0; j <= degree; j++)
			{
				CurveVertex@ p = @v[span - degree + j];
				const float der = ders[i][j];
				
				cd.x += p.x * der;
				cd.y += p.y * der;
				cd.weight += p.weight * der;
			}
		}
	}
	
	void curve_derivatives_rational(const int num_ders, const int degree, const float u)
	{
		const int v_count = closed ? vertex_count + 3 : vertex_count;
		
		if(int(curve_ders.length) < num_ders + 1)
		{
			curve_ders.resize(num_ders + 1);
		}
		
		// Compute homogenous coordinates of control points.
		if(int(v.length) < v_count)
		{
			v.resize(v_count);
		}
		for(int i = 0; i < v_count; i++)
		{
			const CurveVertex@ p = @vertices[i % vertex_count];
			CurveVertex@ vp = @v[i];
			vp.x = p.x * p.weight;
			vp.y = p.y * p.weight;
			vp.weight = p.weight;
		}
		
		// Derivatives of Cw.
		curve_derivatives(num_ders, degree, u);
		
		// Split into coordinates and weights.
		if(v_ders.length < curve_wders.length)
		{
			v_ders.resize(v.length);
			w_ders.resize(v.length);
		}
		for(uint i = 0; i < curve_wders.length; i++)
		{
			const CurveVertex@ vp = @curve_wders[i];
			Point@ vdp = v_ders[i];
			
			vdp.x = vp.x;
			vdp.y = vp.y;
			w_ders[i] = vp.weight;
		}
		
		// Compute rational derivatives.
		const float w0 = w_ders[0];
		for(int i = 0; i <= num_ders; i++)
		{
			Point@ v = v_ders[i];
			for(int j = 1; j <= i; j++)
			{
				CurveVertex@ cd = @curve_ders[i - j];
				const float w = w_ders[j];
				const float binomial = b_spline_binomial(i, j) * w;
				v.x -= binomial * cd.x;
				v.y -= binomial * cd.y;
			}
			
			CurveVertex@ cd = @curve_ders[i];
			cd.x = v.x / w0;
			cd.y = v.y / w0;
		}
	}
	
	void calc_b_spline_tangent(const float t, float &out tangent_x, float &out tangent_y)
	{
		const int v_count = closed ? vertex_count + 3 : vertex_count;
		const int clamp_val = b_spline_clamped && !closed ? 0 : 1;
		const int degree = clamp(b_spline_degree, 2, v_count - 1);
		const float u = clamp_val + t * (v_count - degree - clamp_val);
		
		curve_derivatives_rational(1, degree, u);

		CurveVertex@ du = @curve_ders[1];
		tangent_x = du.x;
		tangent_y = du.y;
		
		const float length = sqrt(tangent_x * tangent_x + tangent_y * tangent_y);
		if(length != 0)
		{
			tangent_x /= length;
			tangent_y /= length;
		}
	}
	
	private int b_spline_binomial(const int n, const int k)
	{
		int result = 1;
		if(k > n)
			return 0;
		
		for(int i = 1; i <= k; ++i)
		{
			result *= (n + 1 - i);
			result /= i;
		}
		return result;
	}
	
	private int b_spline_find_span(const int degree, const float u)
	{
		// Index of last control point.
		const int n = knots_length - degree - 2;
		// For values of u that lies outside the domain
		if (u >= knots[n + 1])
			return n;
		if (u <= knots[degree])
			return degree;
		
		// Binary search
		int low = degree;
		int high = n + 1;
		int mid = (low + high) / 2;
		
		while(u < knots[mid] || u >= knots[mid + 1])
		{
			if(u < knots[mid])
			{
				high = mid;
			}
			else
			{
				low = mid;
			}
			
			mid = (low + high) / 2;
		}
		
		return mid;
	}
	
	private void b_spline_basis(const int degree, const int span, const float u)
	{
		const int size = degree + 1;
		
		if(int(n.length) < size)
		{
			n.resize(size);
		}
		if(int(left.length) < size)
		{
			left.resize(size);
		}
		if(int(right.length) < size)
		{
			right.resize(size);
		}
		
		float saved = 0;
		//float temp = 0;
		
		for(int i = 0; i < size; i++)
		{
			left[i] = 0;
			right[i] = 0;
			n[i] = 0;
		}
		
		n[0] = 1.0;
		
		for(int j = 1; j <= degree; j++)
		{
			left[j] = u - knots[span + 1 - j];
			right[j] = knots[span + j] - u;
			saved = 0.0;
			
			for(int r = 0; r < j; r++)
			{
				const float temp = n[r] / (right[r + 1] + left[j - r]);
				n[r] = saved + right[r + 1] * temp;
				saved = left[j - r] * temp;
			}
			n[j] = saved;
		}
	}
	
	private void b_spline_der_basis(const int degree, const int span, const float u, const int num_ders)
	{
		const int size = degree + 1;
		
		if(int(left.length) < size)
		{
			left.resize(size);
		}
		if(int(right.length) < size)
		{
			right.resize(size);
		}
		
		float saved = 0;
		float temp = 0;
		
		ensure_array_2(@ndu, size, size);
		
		ndu[0][0] = 1;
		
		for(int i = 1; i <= degree; i++)
		{
			left[i] = u - knots[span + 1 - i];
			right[i] = knots[span + i] - u;
			saved = 0.0;

			for(int j = 0; j < i; j++)
			{
				// Lower triangle
				ndu[i][j] = right[j + 1] + left[i - j];
				temp = ndu[j][i - 1] / ndu[i][j];
				// Upper triangle
				ndu[j][i] = saved + right[j + 1] * temp;
				saved = left[i - j] * temp;
			}

			ndu[i][i] = saved;
		}
		
		ensure_array_2(@ders, num_ders + 1, degree + 1);
		for(int i = 0; i <= degree; i++)
		{
			ders[0][i] = ndu[i][degree];
		}
		
		ensure_array_2(@b_a,2, degree + 1);
		
		for(int r = 0; r <= degree; r++)
		{
			int s1 = 0;
			int s2 = 1;
			b_a[0][0] = 1.0;
			
			for(int k = 1; k <= num_ders; k++)
			{
				const int rk = r - k;
				const int pk = degree - k;
				float d = 0.0;
				int j1 = 0;
				int j2 = 0;
				
				if(r >= k)
				{
					b_a[s2][0] = b_a[s1][0] / ndu[pk + 1][rk];
					d = b_a[s2][0] * ndu[rk][pk];
				}
				
				j1 = rk >= -1 ? 1 : -rk;
				j2 = r - 1 <= pk ? k - 1 : degree - r;
				
				for(int j = j1; j <= j2; j++)
				{
					b_a[s2][j] = (b_a[s1][j] - b_a[s1][j - 1]) / ndu[pk + 1][rk + j];
					d += b_a[s2][j] * ndu[rk + j][pk];
				}
				
				if(r <= pk)
				{
					b_a[s2][k] = -b_a[s1][k - 1] / ndu[pk + 1][r];
					d += b_a[s2][k] * ndu[r][pk];
				}
				
				ders[k][r] = d;
				
				const int tempi = s1;
				s1 = s2;
				s2 = tempi;
			}
		}
		
		float fac = degree;
		for(int k = 1; k <= num_ders; k++)
		{
			for (int j = 0; j <= degree; j++)
			{
				ders[k][j] *= fac;
			}
			
			fac *= degree - k;
		}
	}
	
	private void ensure_array_2(array<array<float>>@ arr, const uint n1, const uint n2)
	{
		if(arr.length >= n1)
			return;
		
		const uint si = arr.length;
		arr.resize(n1);
		
		for(uint i = si; i < n1; i++)
		{
			if(arr[i].length < n2)
			{
				arr[i].resize(n2);
			}
		}
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
