#include 'BSplineEvaluator.cpp';
#include 'CurvePoint.cpp';
#include 'CurveTypes.cpp';
#include 'CurveVertex.cpp';

class BaseCurve
{
	
	// TODO: control_point_start/end should always be moved relative to the start/end vertices
	//       when `end_controls` is not `Manual`.
	// TODO: Calculate bounding box for entire curve and per segment.
	// TODO: Curve x/y, scale x/y, and rotation.
	
	[option,Linear,QuadraticBezier,CubicBezier,CatmullRom,BSpline]
	private CurveType _type = CubicBezier;
	
	/// Do not set directly.
	[persist] int vertex_count;
	
	/// Do not set directly.
	[persist] array<CurveVertex> vertices;
	
	[persist] CurveEndControl _end_controls = AutomaticAngle;
	
	/// Only applicable for CatmullRom splines with `end_controls` set to `Manual`.
	[persist] CurveVertex control_point_start;
	
	/// Only applicable for CatmullRom splines with `end_controls` set to `Manual`.
	[persist] CurveVertex control_point_end;
	
	/// If true the start and end points of this path will automatically connect to each other.
	[persist] private bool _closed;
	
	/// Controls the global tension for CatmullRom splines.
	[persist] float tension = 1;
	
	/// The smoothness of the b-spline. Must be > 1 and < `vertex_count`.
	[persist] private int _b_spline_degree = 2;
	
	/// If true the curve will pass touch the first and last vertices.
	[persist] private bool _b_spline_clamped = true;
	
	/// The total (approximate) length of this curve.
	/// Is only valid after the `update` is called.
	float length;
	
	
	/// One or more segments on this curve have been changed and need to be updated.
	private bool invalidated = true;
	
	/// The knot vector requires regneration after vertices are added/removed.
	private bool invalidated_knots = true;
	
	private BSplineEvaluator@ b_spline;
	
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
			
			init_bezier_control_points();
		}
	}
	
	bool closed
	{
		get const { return _closed; }
		set
		{
			if(_closed == value)
				return;
			
			_closed = value;
			invalidate_b_spline(true);
		}
	}
	
	int b_spline_degree
	{
		get const { return _b_spline_degree; }
		set
		{
			if(_b_spline_degree == value)
				return;
			
			_b_spline_degree = value;
			invalidate_b_spline(true);
		}
	}
	
	bool b_spline_clamped
	{
		get const { return _b_spline_clamped; }
		set
		{
			if(_b_spline_clamped == value)
				return;
			
			_b_spline_clamped = value;
			invalidate_b_spline(true);
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
	
	// --
	
	void clear()
	{
		vertices.resize(0);
		vertex_count = 0;
		
		control_point_start.type = None;
		control_point_end.type = None;
		
		invalidate();
	}
	
	void add_vertex(const float x, const float y)
	{
		vertices.insertLast(CurveVertex(x, y));
		vertex_count++;
		
		init_bezier_control_points();
		
		invalidated = true;
		invalidate_b_spline(true);
	}
	
	/// Call after changing anything about this curve to trigger an `update`.
	/// If no index is passed, the all vertices/segments on the curve will be update/recalculated.
	void invalidate(const int vertex_index=-1)
	{
		invalidated = true;
		
		const int start = vertex_index < 0 ? 0 : vertex_index;
		const int end = vertex_index < 0 ? vertex_count : min(vertex_index, vertex_count);
		
		for(int i = start; i < end; i++)
		{
			vertices[i].invalidated = true;
		}
		
		invalidate_b_spline();
	}
	
	/// Call to update/calcualte some simple initial positions for new control points.
	/// Make sure to call this or manually set control points after adding vertices as control points default to NAN.
	/// If `force` is true all control points will be recalculated, otherwise only newly added ones will be.
	void init_bezier_control_points(const bool force=false, const int from_index=0, const int count=0xffffff)
	{
		switch(_type)
		{
			case CurveType::CubicBezier:
				init_cubic_bezier_control_points(force, from_index, count);
				break;
			case CurveType::QuadraticBezier:
				init_quadratic_bezier_control_points(force, from_index, count);
				break;
		}
	}
	
	/// Use `init_bezier_control_points` instead to intialise control points for the current type.
	void init_cubic_bezier_control_points(const bool force=false, const int from_index=0, const int count=0xffffff)
	{
		if(vertex_count <= 1)
			return;
		
		const int end = from_index + count < vertex_count ? from_index + count : vertex_count;
		for(int i = from_index; i < end; i++)
		{
			CurveVertex@ p1 = @vertices[i];
			CurveControlPoint@ cp1 = p1.cubic_control_point_1;
			CurveControlPoint@ cp2 = p1.cubic_control_point_2;
			
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
	
	/// Use `init_bezier_control_points` instead to intialise control points for the current type.
	void init_quadratic_bezier_control_points(const bool force=false, const int from_index=0, const int count=0xffffff)
	{
		const int end = from_index + count < vertex_count ? from_index + count : vertex_count;
		for(int i = from_index; i < end; i++)
		{
			CurveVertex@ p1 = @vertices[i];
			CurveControlPoint@ cp = p1.quad_control_point;
			
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
	
	/// Evaluate the curve at the given t value and return the position and normal.
	void eval(
		const float t, float &out x, float &out y, float &out normal_x, float &out normal_y,
		const EvalReturnType return_type=EvalReturnType::Both)
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
				eval_linear(t, x, y, normal_x, normal_y, return_type);
				break;
			case QuadraticBezier:
				eval_quadratic_bezier(t, x, y, normal_x, normal_y, return_type);
				break;
			case CubicBezier:
				eval_cubic_bezier(t, x, y, normal_x, normal_y, return_type);
				break;
			case CatmullRom:
				eval_catmull_rom(t, x, y, normal_x, normal_y, return_type);
				break;
			case BSpline:
				eval_b_spline(t, x, y, normal_x, normal_y, return_type);
				break;
			default:
				x = 0;
				y = 0;
				normal_x = 1;
				normal_y = 0;
				break;
		}
	}
	
	void eval_linear(
		const float t, float &out x, float &out y, float &out normal_x, float &out normal_y,
		const EvalReturnType return_type=EvalReturnType::Both)
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
		
		if(return_type == EvalReturnType::Both || return_type == EvalReturnType::Point)
		{
			x = p1.x + dx * ti;
			y = p1.y + dy * ti;
		}
		
		// Calculate the normal vector.
		if(return_type == EvalReturnType::Both || return_type == EvalReturnType::Normal)
		{
			const float length = sqrt(dx * dx + dy * dy);
			if(length != 0)
			{
				normal_x = dy / length;
				normal_y = -dx / length;
			}
			else
			{
				normal_x = normal_y = 0;
			}
		}
	}
	
	/// https://pomax.github.io/bezierinfo/#catmullconv
	void eval_catmull_rom(
		const float t, float &out x, float &out y, float &out normal_x, float &out normal_y,
		const EvalReturnType return_type=EvalReturnType::Both)
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
		const CurveControlPoint@ p0 = p1.type != Square
			? closed || i > 0
				? vert(i, -1)
				: _end_controls != Manual
					? this.p0.extrapolate(p1, p2,
						_end_controls == CurveEndControl::AutomaticAngle && vertex_count >= 3 ? @vertices[2] : null)
					: check_control_point_start()
			: p1;
		const CurveControlPoint@ p3 = p2.type != Square
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
		
		if(return_type == EvalReturnType::Both || return_type == EvalReturnType::Point)
		{
			const float c0 = 2 * t3 - 3 * t2 + 1;
			const float c1 = t3 - 2 * t2 + ti;
			const float c2 = -2 * t3 + 3 * t2;
			const float c3 = t3 - t2;
			x = c0 * p1.x + c1 * dv1x + c2 * p2.x + c3 * dv2x;
			y = c0 * p1.y + c1 * dv1y + c2 * p2.y + c3 * dv2y;
		}
		
		// Calculate the normal.
		if(return_type == EvalReturnType::Both || return_type == EvalReturnType::Normal)
		{
			normal_x =
				(3 * t2 - 4 * ti + 1) * dv1y +
				(3 * t2 - 2 * ti) * dv2y +
				p1.y * (6 * t2 - 6 * ti) + p2.y * (6 * ti - 6 * t2);
			normal_y = -(
				(3 * t2 - 4 * ti + 1) * dv1x +
				(3 * t2 - 2 * ti) * dv2x +
				p1.x * (6 * t2 - 6 * ti) + p2.x * (6 * ti - 6 * t2));
			
			const float length = sqrt(normal_x * normal_x + normal_y * normal_y);
			if(length != 0)
			{
				normal_x /= length;
				normal_y /= length;
			}
		}
	}
	
	void eval_quadratic_bezier(
		const float t, float &out x, float &out y, float &out normal_x, float &out normal_y,
		const EvalReturnType return_type=EvalReturnType::Both)
	{
		int i;
		float ti;
		calc_segment_t(t, ti, i);
		
		// Get vertices.
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p2 = vert(i, 1);
		const CurveControlPoint@ cp = p1.quad_control_point;
		const float cpx = p1.x + cp.x;
		const float cpy = p1.y + cp.y;
		
		// Calculate the point.
		const float u = 1 - ti;
		const float tt = ti * ti;
		const float uu = u * u;
		const float ut2 = 2 * u * ti;
		
		// Normal
		if(cp.weight == 1)
		{
			if(return_type == EvalReturnType::Both || return_type == EvalReturnType::Point)
			{
				x = uu * p1.x + ut2 * cpx + tt * p2.x;
				y = uu * p1.y + ut2 * cpy + tt * p2.y;
			}
			
			if(return_type == EvalReturnType::Both || return_type == EvalReturnType::Normal)
			{
				normal_x =  2 * (u * (cpy - p1.y) + ti * (p2.y - cpy));
				normal_y = -2 * (u * (cpx - p1.x) + ti * (p2.x - cpx));
			}
		}
		// Conic/Weighted
		else
		{
			const float f0 = p1.weight * uu;
			const float f1 = cp.weight * ut2;
			const float f2 = p2.weight * tt;
			const float basis = f0 + f1 + f2;
			
			if(return_type == EvalReturnType::Both || return_type == EvalReturnType::Point)
			{
				x = (f0 * p1.x + f1 * cpx + f2 * p2.x) / basis;
				y = (f0 * p1.y + f1 * cpy + f2 * p2.y) / basis;
			}
			
			if(return_type == EvalReturnType::Both || return_type == EvalReturnType::Normal)
			{
				const float basis2 = -2 * p1.weight * u + 2 * cp.weight * u - 2 * cp.weight * ti + 2 * p2.weight * ti;
				normal_x = (
					-2 * p1.weight * p1.y * u + 2 * cp.weight * cpy * u - 2 * cp.weight * cpy * ti + 2 * p2.weight * p2.y * ti) / basis -
					(basis2 * (p1.weight * p1.y * uu + 2 * cp.weight * cpy * ti * u + p2.weight * p2.y * tt)) / (basis * basis);
				normal_y = -((
					-2 * p1.weight * p1.x * u + 2 * cp.weight * cpx * u - 2 * cp.weight * cpx * ti + 2 * p2.weight * p2.x * ti) / basis -
					(basis2 * (p1.weight * p1.x * uu + 2 * cp.weight * cpx * ti * u + p2.weight * p2.x * tt)) / (basis * basis));
			}
		}
		
		const float length = sqrt(normal_x * normal_x + normal_y * normal_y);
		if(length != 0)
		{
			normal_x /= length;
			normal_y /= length;
		}
	}
	
	void eval_cubic_bezier(
		const float t, float &out x, float &out y, float &out normal_x, float &out normal_y,
		const EvalReturnType return_type=EvalReturnType::Both)
	{
		int i;
		float ti;
		calc_segment_t(t, ti, i);
		int i2 = i * 2;
		
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p2 = vert(i, 1);
		const CurveControlPoint@ cp1 = p1.type != Square
			? p1.cubic_control_point_2
			: p1;
		const CurveControlPoint@ cp2 = p2.type != Square
			? p2.cubic_control_point_1
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
		
		// Normal
		if(p1.weight == 1 && p2.weight == 1 && cp1.weight == 1 && cp2.weight == 1)
		{
			if(return_type == EvalReturnType::Both || return_type == EvalReturnType::Point)
			{
				x = uuu * p1.x + 3 * uu * ti * cp1x + 3 * u * tt * cp2x + tt3 * p2.x;
				y = uuu * p1.y + 3 * uu * ti * cp1y + 3 * u * tt * cp2y + tt3 * p2.y;
			}
			if(return_type == EvalReturnType::Both || return_type == EvalReturnType::Normal)
			{
				normal_x =
					-3 * p1.y * uu +
					3 * cp1y * (3 * uu - 2 * u) +
					3 * cp2y * (2 * ti - 3 * tt) +
					3 * p2.y * tt;
				normal_y = -(
					-3 * p1.x * uu +
					3 * cp1x * (3 * uu - 2 * u) +
					3 * cp2x * (2 * ti - 3 * tt) +
					3 * p2.x * tt);
			}
		}
		// Weighted
		else
		{
			const float f0 = uuu * p1.weight;
			const float f1 = 3 * uu * ti * cp1.weight;
			const float f2 = 3 * u * tt * cp2.weight;
			const float f3 = tt3 * p2.weight;
			const float basis = f0 + f1 + f2 + f3;
			
			if(return_type == EvalReturnType::Both || return_type == EvalReturnType::Point)
			{
				x = (f0 * p1.x + f1 * cp1x + f2 * cp2x + f3 * p2.x) / basis;
				y = (f0 * p1.y + f1 * cp1y + f2 * cp2y + f3 * p2.y) / basis;
			}
			
			if(return_type == EvalReturnType::Both || return_type == EvalReturnType::Normal)
			{
				const float prime =
					-3 * p1.weight * uu +
					 3 * cp1.weight * uu
					-6 * cp1.weight * ti * u
					-3 * cp2.weight * tt +
					 6 * cp2.weight * ti * u +
					 3 * p2.weight * tt;
				normal_x =
					(
						-3 * p1.y * p1.weight * uu +
						3 * cp1y * cp1.weight * uu -
						6 * cp1y * cp1.weight * ti * u -
						3 * cp2y * cp2.weight * tt +
						6 * cp2y * cp2.weight * ti * u +
						3 * p2.y * p2.weight * tt
					) / basis
					- (prime * (p1.y * f0 + cp1y * f1 + cp2y * f2 + p2.y * f3))
					/ (basis * basis);
				normal_y = -(
					(
						-3 * p1.x * p1.weight * uu +
						3 * cp1x * cp1.weight * uu -
						6 * cp1x * cp1.weight * ti * u -
						3 * cp2x * cp2.weight * tt +
						6 * cp2x * cp2.weight * ti * u +
						3 * p2.x * p2.weight * tt
					) / basis
					- (prime * (p1.x * f0 + cp1x * f1 + cp2x * f2 + p2.x * f3)
					) / (basis * basis));
			}
		}
		
		if(return_type == EvalReturnType::Both || return_type == EvalReturnType::Normal)
		{
			const float length = sqrt(normal_x * normal_x + normal_y * normal_y);
			if(length != 0)
			{
				normal_x /= length;
				normal_y /= length;
			}
		}
	}
	
	void eval_b_spline(
		const float t, float &out x, float &out y, float &out normal_x, float &out normal_y,
		const EvalReturnType return_type=EvalReturnType::Both)
	{
		if(b_spline_degree <= 1)
		{
			eval_linear(t, x, y, normal_x, normal_y);
			return;
		}
		
		if(@b_spline == null)
		{
			@b_spline = BSplineEvaluator();
		}
		
		if(invalidated_knots)
		{
			b_spline.generate_knots(@vertices, vertex_count, b_spline_degree, b_spline_clamped, closed);
			invalidated_knots = false;
		}
		
		b_spline.eval(
			t, x, y, normal_x, normal_y,
			@vertices, vertex_count,
			b_spline_degree, b_spline_clamped, closed, return_type);
		
		if(return_type == EvalReturnType::Both || return_type == EvalReturnType::Normal)
		{
			const float length = sqrt(normal_x * normal_x + normal_y * normal_y);
			if(length != 0)
			{
				normal_x /= length;
				normal_y /= length;
			}
		}
	}
	
	// 
	
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
	
	CurveVertex@ vert(const int i, const int offset=0)
	{
		return vertices[((i + offset) % vertex_count + vertex_count) % vertex_count];
	}
	
	// --
	
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
	
	private void invalidate_b_spline(const bool also_invalidate_knots=false)
	{
		if(also_invalidate_knots)
		{
			invalidated_knots = true;
		}
		
		if(@b_spline != null)
		{
			b_spline.invalidate_vertices();
		}
	}
	
	/// Recalculates all values on all invalidated segments when necessary.
	private void update()
	{
		if(!invalidated)
			return;
		
		for(int i = 0; i < vertex_count; i++)
		{
			CurveVertex@ v = vertices[i];
			
			if(v.invalidated)
			{
				
				v.invalidated = false;
			}
		}
		
		invalidated = false;
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
