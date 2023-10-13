#include 'BSpline.cpp';
#include 'CatmullRom.cpp';
#include 'CubicBezier.cpp';
#include 'CubicBezierRational.cpp';
#include 'QuadraticBezier.cpp';
#include 'QuadraticBezierRational.cpp';

#include 'calculate_arc_lengths.cpp';
#include 'CurveVertex.cpp';

/** A higher level wrapper designed for editing/manipulating curves with support for several
  * different types as well as chaining multiple curves together. */
class MultiCurve
{
	
	// TODO: Option to not automatically calculate arc lengths.
	// TODO: Debugdraw
	// 		- View bounds - don't draw things outside of this
	// 			- Aslo check individual arc/adaptive segments? This wouldn't be 100% reliable since an arc segment can extend beyound the p1>p2 bounding box.
	// 				- Could have some kind of padding (relative to arc length?)
	// 			- Find t/view intersections and only draw between those t values?
	// TODO: Only invalidate vertices/segments that change.
	// TODO: Both `invalidate` and `validate` seem kind of redundant?
	// TODO: Option/method to calculate simple and complex bounding boxes (using newtons method for rational curves)
	// TODO: When editing non-quadratic, the quadratic control points can potentially get very far away from the vertices
	//       so maybe storing the absolutely is not a good idea?
	//       OR when moving vertices, try interpolate and move the control point based on the two vertices.
	// TODO: control_point_start/end should always be moved relative to the start/end vertices
	//       when `end_controls` is not `Manual`.
	// TODO: ? Add basic CurveEditor class
	
	[option,Linear,QuadraticBezier,CubicBezier,CatmullRom,BSpline]
	private CurveType _type = CubicBezier;
	
	/** Do not set directly. */
	[persist] int vertex_count;
	
	/** Do not set directly. */
	[persist] array<CurveVertex> vertices;
	
	[persist] CurveEndControl _end_controls = AutomaticAngle;
	
	/** Only applicable for CatmullRom splines with `end_controls` set to `Manual`. */
	[persist] CurveVertex control_point_start;
	
	/** Only applicable for CatmullRom splines with `end_controls` set to `Manual`. */
	[persist] CurveVertex control_point_end;
	
	/** If true the start and end points of this path will automatically connect to each other. */
	[persist] private bool _closed;
	
	/** Controls the global tension for CatmullRom splines. */
	[persist] float tension = 1;
	
	/** The smoothness of the b-spline. Must be > 1 and < `vertex_count`. */
	[persist] private int _b_spline_degree = 2;
	
	/** If true the curve will pass touch the first and last vertices. */
	[persist] private bool _b_spline_clamped = true;
	
	/** Controls the base number of the pre-calcualted subdivisions of this curve.
	  * These subdivisions are required for certain operations, e.g. approximating the curve length, or finding
	  * a closest point on the curve.
	  * The default values skew towards accuracy over performance.
	  * Changes to these values will only take effect when the curve is next validated.
	  * See `Curve::calculate_arc_lengths` for descriptions of these properties. */
	int subdivision_count = 6;
	
	/** See `Curve::calculate_arc_lengths` `max_length`. */
	float max_division_length = 0;
	
	/** See `Curve::calculate_arc_lengths`. */
	float adaptive_angle = 4;
	
	/** See `Curve::calculate_arc_lengths`. */
	int adaptive_max_subdivisions = 4;
	
	/** See `Curve::calculate_arc_lengths`. */
	float adaptive_min_length = 0;
	
	/** See `Curve::calculate_arc_lengths`. */
	float adaptive_angle_max = 65;
	
	/** See `Curve::calculate_arc_lengths`. */
	float adaptive_length_factor = 0.1;
	
	/** The total (approximate) length of this curve. */
	float length;
	
	/** This curve's bounding box. */
	float x1, y1, x2, y2;
	
	// --
	
	/** One or more segments on this curve have been changed and need to be updated. */
	private bool invalidated = true;
	
	/** The b-spline knot vector requires regneration after vertices are added/removed. */
	private bool invalidated_b_spline_vertices = true;
	
	/** The b-spline knot vector requires regneration after vertices are added/removed. */
	private bool invalidated_b_spline_knots = true;
	
	private BSpline@ b_spline;
	
	/** Temp points used when calculating automatic end control points. */
	private CurveVertex p0;
	private CurveVertex p3;
	private CurveArc _ct1, _ct2;
	
	private Curve::EvalFunc@ eval_func_def;
	
	MultiCurve()
	{
		control_point_start.type = None;
		control_point_end.type = None;
		
		@eval_func_def = Curve::EvalFunc(eval);
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
			
			if(_type == BSpline && @b_spline == null)
			{
				@b_spline = BSpline();
			}
			
			init_bezier_control_points();
			
			invalidated = true;
			invalidated_b_spline_knots = true;
			invalidated_b_spline_vertices = true;
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
			
			invalidated = true;
			invalidated_b_spline_knots = true;
			invalidated_b_spline_vertices = true;
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
			
			invalidated = true;
			invalidated_b_spline_knots = true;
			invalidated_b_spline_vertices = true;
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
			
			if(!closed)
			{
				invalidated = true;
				invalidated_b_spline_knots = true;
				invalidated_b_spline_vertices = true;
			}
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
	
	const float segment_index_max
	{
		get const { return closed ? vertex_count - 1 : vertex_count - 2; }
	}
	
	// --
	
	void clear()
	{
		vertices.resize(0);
		vertex_count = 0;
		
		control_point_start.type = None;
		control_point_end.type = None;
		
		invalidated = true;
		invalidated_b_spline_knots = true;
		invalidated_b_spline_vertices = true;
	}
	
	CurveVertex@ add_vertex(const float x, const float y)
	{
		vertices.insertLast(CurveVertex(x, y));
		vertex_count++;
		
		init_bezier_control_points(false, vertex_count - 1, 1);
		
		invalidated = true;
		invalidated_b_spline_knots = true;
		invalidated_b_spline_vertices = true;
		
		return @vertices[vertex_count - 1];
	}
	
	/** Call after modifying this curve in any way, so that cached values such as lengths, bounding boxes, etc. can be recalculated.
	  * Passing a vertex index in will invalidate only that vertex, potentially reducing the number of calculations.
	  * Certain operation such as adding vertices, changing the curve type, etc. will automatically trigger invalidation, but directly setting properties
	  * such as vertex position will require manually calling `invalidate`. */
	void invalidate()
	{
		invalidated = true;
		invalidated_b_spline_vertices = true;
	}
	
	/** Invalidates a single vertex, potentially reducing the number of calculations. */
	void invalidate(const int start_index, const int end_index=-1)
	{
		invalidated = true;
		
		const int i2 = end_index < 0 ? start_index : end_index;
		for(int i = start_index; i <= end_index; i++)
		{
			
		}
	}
	
	/** Must be called after `invalidate` and any time the curve is modified in any way.
	  * Recalculates cached values such as the bounding box, curve length, etc. */
	void validate()
	{
		if(!invalidated)
			return;
		
		// -- Update BSplines.
		
		if(_type == CurveType::BSpline)
		{
			if(invalidated_b_spline_vertices)
			{
				b_spline.set_vertices(@vertices, vertex_count, _b_spline_degree, _b_spline_clamped, _closed);
				invalidated_b_spline_vertices = false;
			}
			if(invalidated_b_spline_knots)
			{
				b_spline.generate_knots(_b_spline_degree, _b_spline_clamped, _closed);
				invalidated_b_spline_knots = false;
			}
		}
		
		// -- Calculate the bounding box.
		
		x1 = INFINITY;
		y1 = INFINITY;
		x2 = -INFINITY;
		y2 = -INFINITY;
		
		switch(_type)
		{
			case CurveType::CatmullRom:
				calc_bounding_box_catmull_rom();
				break;
			case CurveType::QuadraticBezier:
				calc_bounding_box_quadratic_bezier();
				break;
			case CurveType::CubicBezier:
				calc_bounding_box_cubic_bezier();
				break;
			case CurveType::BSpline:
				calc_bounding_box_b_spline();
				break;
			case CurveType::Linear:
			default:
				calc_bounding_box_linear();
				break;
		}
		
		// -- Calculate arc lengths.
		
		//puts('--------------------------');
		length = Curve::calculate_arc_lengths(
			@vertices, vertex_count, _closed,
			eval_func_def, _type != Linear ? subdivision_count : 2, max_division_length,
			_type != Linear ? adaptive_angle * DEG2RAD : 0,
			adaptive_max_subdivisions, adaptive_min_length, adaptive_angle_max * DEG2RAD, adaptive_length_factor);
		//length = Curve::calculate_arc_lengths(
		//	@vertices, vertex_count, _closed,
		//	eval_func_def, _type != Linear ? subdivision_count : 2);
		
		invalidated = false;
	}
	
	/** Call to update/calculate some simple initial positions for new control points. Mostly for testing.
	  * Make sure to call this or manually set control points after adding vertices as control points default to NAN.
	  * If `force` is true all control points will be recalculated, otherwise only newly added ones will be. */
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
	
	void init_quadratic_bezier_control_points(const bool force=false, const int from_index=0, const int count=0xffffff)
	{
		const int end = from_index + count <= vertex_count ? from_index + count : vertex_count;
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
				cp.x = p1.x + tx * 0.5;
				cp.y = p1.y + ty * 0.5;
			}
		}
	}
	
	// -- Eval methods --
	
	/** Calculate the position and normal at the given segment and t value.
	 * To return evenly spaced points along the curve, use the mapping methods to convert a distance to a t value.
	 * @param segment The index of the segment between 0 and `vertex_count` for closed curves, and `vertex_count` for open.
	 *                Passing a negative values will instead calculate the segment index automatically, and `t` will be considered
	 *                an absolute value with 0 being the first vertex of the curve, and 1 being the last.
	 * @param t The factor between 0 and 1 within `segment`, or the entire curve if `segment` is negative.
	 * @param normalise If false the returned normal values will not be normalised. */
	void eval(
		const int segment, const float t, float &out x, float &out y, float &out normal_x, float &out normal_y,
		const bool normalise=true)
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
			case CurveType::Linear:
				eval_linear(segment, t, x, y, normal_x, normal_y, normalise);
				break;
			case CurveType::QuadraticBezier:
				eval_quadratic_bezier(segment, t, x, y, normal_x, normal_y, normalise);
				break;
			case CurveType::CubicBezier:
				eval_cubic_bezier(segment, t, x, y, normal_x, normal_y, normalise);
				break;
			case CurveType::CatmullRom:
				eval_catmull_rom(segment, t, x, y, normal_x, normal_y, normalise);
				break;
			case CurveType::BSpline:
				eval_b_spline(segment, t, x, y, normal_x, normal_y, normalise);
				break;
			default:
				x = 0;
				y = 0;
				normal_x = 1;
				normal_y = 0;
				break;
		}
	}
	
	/** Calculate the position a the given segment and t value. */
	void eval_point(const int segment, const float t, float &out x, float &out y)
	{
		if(vertex_count == 0)
		{
			x = 0;
			y = 0;
			return;
		}
		if(vertex_count == 1)
		{
			const CurveVertex@ p0 = @vertices[0];
			x = p0.x;
			y = p0.y;
			return;
		}
		
		switch(_type)
		{
			case CurveType::Linear:
				eval_linear_point(segment, t, x, y);
				break;
			case CurveType::QuadraticBezier:
				eval_quadratic_bezier_point(segment, t, x, y);
				break;
			case CurveType::CubicBezier:
				eval_cubic_bezier_point(segment, t, x, y);
				break;
			case CurveType::CatmullRom:
				eval_catmull_rom_point(segment, t, x, y);
				break;
			case CurveType::BSpline:
				eval_b_spline_point(segment, t, x, y);
				break;
			default:
				x = 0;
				y = 0;
				break;
		}
	}
	
	/** Calculate the normal a the given segment and t value. */
	void eval_normal(const int segment, const float t, float &out normal_x, float &out normal_y, const bool normalise=true)
	{
		if(vertex_count <= 1)
		{
			normal_x = 1;
			normal_y = 0;
			return;
		}
		
		switch(_type)
		{
			case CurveType::Linear:
				eval_linear_normal(segment, t, normal_x, normal_y, normalise);
				break;
			case CurveType::QuadraticBezier:
				eval_quadratic_bezier_normal(segment, t, normal_x, normal_y, normalise);
				break;
			case CurveType::CubicBezier:
				eval_cubic_bezier_normal(segment, t, normal_x, normal_y, normalise);
				break;
			case CurveType::CatmullRom:
				eval_catmull_rom_normal(segment, t, normal_x, normal_y, normalise);
				break;
			case CurveType::BSpline:
				eval_b_spline_normal(segment, t, normal_x, normal_y, normalise);
				break;
			default:
				normal_x = 1;
				normal_y = 0;
				break;
		}
	}
	
	void eval_linear(
		const int segment, const float t, float &out x, float &out y, float &out normal_x, float &out normal_y,
		const bool normalise=true)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		// Get vertices.
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p2 = vert(i, 1);
		
		// Calculate the point.
		const float dx = p2.x - p1.x;
		const float dy = p2.y - p1.y;
		
		x = p1.x + dx * ti;
		y = p1.y + dy * ti;
		
		// Calculate the normal vector.
		if(normalise)
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
		else
		{
			normal_x = dy;
			normal_y = -dx;
		}
	}
	
	void eval_linear_point(const int segment, const float t, float &out x, float &out y)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		// Get vertices.
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p2 = vert(i, 1);
		
		// Calculate the point.
		const float dx = p2.x - p1.x;
		const float dy = p2.y - p1.y;
		
		x = p1.x + dx * ti;
		y = p1.y + dy * ti;
	}
	
	void eval_linear_normal(const int segment, const float t, float &out normal_x, float &out normal_y, const bool normalise=true)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		// Get vertices.
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p2 = vert(i, 1);
		
		// Calculate the normal vector.
		normal_y = -(p2.x - p1.x);
		normal_x = p2.y - p1.y;
		
		if(normalise)
		{
			const float length = sqrt(normal_x * normal_x + normal_y * normal_y);
			if(length != 0)
			{
				normal_x /= length;
				normal_y /= length;
			}
			else
			{
				normal_x = normal_y = 0;
			}
		}
	}
	
	void eval_catmull_rom(
		const int segment, const float t, float &out x, float &out y, float &out normal_x, float &out normal_y,
		const bool normalise=true)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		CurveVertex@ p2, p3;
		CurveControlPoint@ p1, p4;
		get_segment_catmull_rom(i, p1, p2, p3, p4);
		
		CatmullRom::eval(
			p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y,
			tension * p2.tension,
			ti, x, y, normal_x, normal_y, normalise);
	}
	
	void eval_catmull_rom_point(const int segment, const float t, float &out x, float &out y)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		CurveVertex@ p2, p3;
		CurveControlPoint@ p1, p4;
		get_segment_catmull_rom(i, p1, p2, p3, p4);
		
		CatmullRom::eval_point(
			p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y,
			tension * p2.tension,
			ti, x, y);
	}
	
	void eval_catmull_rom_normal(const int segment, const float t, float &out normal_x, float &out normal_y, const bool normalise=true)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		CurveVertex@ p2, p3;
		CurveControlPoint@ p1, p4;
		get_segment_catmull_rom(i, p1, p2, p3, p4);
		
		CatmullRom::eval_normal(
			p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y,
			tension * p2.tension,
			ti, normal_x, normal_y, normalise);
	}
	
	void eval_quadratic_bezier(
		const int segment, const float t, float &out x, float &out y, float &out normal_x, float &out normal_y,
		const bool normalise=true)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		// Get vertices.
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p3 = vert(i, 1);
		const CurveControlPoint@ p2 = p1.quad_control_point;
		
		// Non-rational.
		if(p1.weight == 1 && p2.weight == 1 && p3.weight == 1)
		{
			QuadraticBezier::eval(
				p1.x, p1.y, p2.x, p2.y, p3.x, p3.y,
				ti, x, y, normal_x, normal_y, normalise);
		}
		// Rational.
		else
		{
			QuadraticBezier::eval(
				p1.x, p1.y, p2.x, p2.y, p3.x, p3.y,
				p1.weight, p2.weight, p3.weight,
				ti, x, y, normal_x, normal_y, normalise);
		}
	}
	
	void eval_quadratic_bezier_point(const int segment, const float t, float &out x, float &out y)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		// Get vertices.
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p3 = vert(i, 1);
		const CurveControlPoint@ p2 = p1.quad_control_point;
		
		// Non-rational.
		if(p1.weight == 1 && p2.weight == 1 && p3.weight == 1)
		{
			QuadraticBezier::eval_point(
				p1.x, p1.y, p2.x, p2.y, p3.x, p3.y,
				ti, x, y);
		}
		// Rational.
		else
		{
			QuadraticBezier::eval_point(
				p1.x, p1.y, p2.x, p2.y, p3.x, p3.y,
				p1.weight, p2.weight, p3.weight,
				ti, x, y);
		}
	}
	
	void eval_quadratic_bezier_normal(const int segment, const float t, float &out normal_x, float &out normal_y, const bool normalise=true)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		// Get vertices.
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p3 = vert(i, 1);
		const CurveControlPoint@ p2 = p1.quad_control_point;
		
		// Non-rational.
		if(p1.weight == 1 && p2.weight == 1 && p3.weight == 1)
		{
			QuadraticBezier::eval_normal(
				p1.x, p1.y, p2.x, p2.y, p3.x, p3.y,
				ti, normal_x, normal_y, normalise);
		}
		// Rational.
		else
		{
			QuadraticBezier::eval_normal(
				p1.x, p1.y, p2.x, p2.y, p3.x, p3.y,
				p1.weight, p2.weight, p3.weight,
				ti, normal_x, normal_y, normalise);
		}
	}
	
	void eval_cubic_bezier(
		const int segment, const float t, float &out x, float &out y, float &out normal_x, float &out normal_y,
		const bool normalise=true)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		CurveVertex@ p1 = @vertices[i];
		CurveVertex@ p4 = vert(i, 1);
		CurveControlPoint@ p2 = p1.type != Square
			? this.p0.added(p1.cubic_control_point_2, p1)
			: p1;
		CurveControlPoint@ p3 = p4.type != Square
			? this.p3.added(p4.cubic_control_point_1, p4)
			: p4;
		
		// Non-rational.
		if(p1.weight == 1 && p2.weight == 1 && p3.weight == 1 && p4.weight == 1)
		{
			CubicBezier::eval(
				p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y,
				ti, x, y, normal_x, normal_y, normalise);
		}
		// Rational.
		else
		{
			CubicBezier::eval(
				p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y,
				p1.weight, p2.weight, p3.weight, p4.weight,
				ti, x, y, normal_x, normal_y, normalise);
		}
	}
	
	void eval_cubic_bezier_point(const int segment, const float t, float &out x, float &out y)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		CurveVertex@ p1 = @vertices[i];
		CurveVertex@ p4 = vert(i, 1);
		CurveControlPoint@ p2 = p1.type != Square
			? this.p0.added(p1.cubic_control_point_2, p1)
			: p1;
		CurveControlPoint@ p3 = p4.type != Square
			? this.p3.added(p4.cubic_control_point_1, p4)
			: p4;
		
		// Non-rational.
		if(p1.weight == 1 && p2.weight == 1 && p3.weight == 1 && p4.weight == 1)
		{
			CubicBezier::eval_point(
				p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y,
				ti, x, y);
		}
		// Rational.
		else
		{
			CubicBezier::eval_point(
				p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y,
				p1.weight, p2.weight, p3.weight, p4.weight,
				ti, x, y);
		}
	}
	
	void eval_cubic_bezier_normal(const int segment, const float t, float &out normal_x, float &out normal_y, const bool normalise=true)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		CurveVertex@ p1 = @vertices[i];
		CurveVertex@ p4 = vert(i, 1);
		CurveControlPoint@ p2 = p1.type != Square
			? this.p0.added(p1.cubic_control_point_2, p1)
			: p1;
		CurveControlPoint@ p3 = p4.type != Square
			? this.p3.added(p4.cubic_control_point_1, p4)
			: p4;
		
		// Non-rational.
		if(p1.weight == 1 && p2.weight == 1 && p3.weight == 1 && p4.weight == 1)
		{
			CubicBezier::eval_normal(
				p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y,
				ti, normal_x, normal_y, normalise);
		}
		// Rational.
		else
		{
			CubicBezier::eval_normal(
				p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y,
				p1.weight, p2.weight, p3.weight, p4.weight,
				ti, normal_x, normal_y, normalise);
		}
	}
	
	void eval_b_spline(
		const int segment, const float t, float &out x, float &out y, float &out normal_x, float &out normal_y,
		const bool normalise=true)
	{
		if(b_spline_degree <= 1)
		{
			eval_linear(segment, t, x, y, normal_x, normal_y);
			return;
		}
		
		const float ta = segment >= 0
			? (segment + (t > 0 ? t : t < 1 ? t : 1)) / (closed ? vertex_count : vertex_count - 1)
			: t;
		b_spline.eval(
			b_spline_degree, b_spline_clamped, closed, 
			ta, x, y, normal_x, normal_y,
			normalise);
	}
	
	void eval_b_spline_point(const int segment, const float t, float &out x, float &out y)
	{
		if(b_spline_degree <= 1)
		{
			eval_linear_point(segment, t, x, y);
			return;
		}
		
		const float ta = segment >= 0
			? (segment + (t > 0 ? t : t < 1 ? t : 1)) / (closed ? vertex_count : vertex_count - 1)
			: t;
		b_spline.eval_point(
			b_spline_degree, b_spline_clamped, closed, 
			ta, x, y);
	}
	
	void eval_b_spline_normal(const int segment, const float t, float &out normal_x, float &out normal_y, const bool normalise=true)
	{
		if(b_spline_degree <= 1)
		{
			eval_linear_normal(segment, t, normal_x, normal_y);
			return;
		}
		
		const float ta = segment >= 0
			? (segment + (t > 0 ? t : t < 1 ? t : 1)) / (closed ? vertex_count : vertex_count - 1)
			: t;
		b_spline.eval_normal(
			b_spline_degree, b_spline_clamped, closed, 
			ta, normal_x, normal_y);
	}
	
	// --
	
	Debug@ db;
	float db_zf = 1;
	int eval_count = 0;
	int eval_count_2 = 0;
	int icap=0;
	bool tf = true;
	/**
	  * @param max_distance If > 0, only points closer than this will be returned. Can also potentially reduce the amount of work needed
	  *   by skipping segments that are out of range with simple bounds checks.
	  * @param threshold When the distance between tested points becomes smaller than this, stop looking.
	  * @param arc_length_interpolation 
	  * @return true if a point was found within `max_distance`
	  */
	bool closest_point(
		const float x, const float y, int &out segment_index, float &out t, float &out px, float &out py,
		const float max_distance=0, float threshold=1,
		const CurveArcInterpolation arc_length_interpolation=None, const bool interpolate_result=false
		, const bool odb=true)
	{
		icap = 0;
		if(odb) eval_count = 0;
		if(odb) eval_count_2 = 0;
		if(vertex_count == 0 || vertices[0].arc_count == 0)
			return false;
		
		const int end = closed ? vertex_count : vertex_count - 1;
		
		if(max_distance > 0 && (
			x < x1 - max_distance || x > x2 + max_distance ||
			y < y1 - max_distance || y > y2 + max_distance))
			return false;
		
		// -- Step 1. Find the closest arc segment.
		
		segment_index = -1;
		int arc_index = -1;
		CurveArc@ clostest_arc = null;
		float dist = INFINITY;
		float dist_a = INFINITY;
		int force_iteration = 0;
		
		for(int i = 0; i < end; i++)
		{
			if(i != 1) continue;
			CurveVertex@ v = vertices[i];
			
			if(max_distance > 0 && (
				x < v.x1 - max_distance || x > v.x2 + max_distance ||
				y < v.y1 - max_distance || y > v.y2 + max_distance))
				continue;
			
			// Start at 1 because the starting point of this segment is the same as the end point of the previous,
			// which has already been tested.
			for(int j = i > 0 && closed ? 1 : 0; j < v.arc_count; j++)
			{
				CurveArc@ c = v.arcs[j];
				
				float c_dist = -1;
				float c_dist_a = -1;
				
				// Project the point onto the current arc segment to find a more accurate initial guess.
				if(j > 0 && arc_length_interpolation == CurveArcInterpolation::Linear)
				{
					CurveArc@ c0 = v.arcs[j - 1];
					float dx = c.x - c0.x;
					float dy = c.y - c0.y;
					
					if(dx != 0 || dy != 0)
					{
						float pt = ((x - c0.x) * dx + (y - c0.y) * dy) / (dx * dx + dy * dy);
						
						if(pt > 0 && pt < 1)
						{
							//_ct1.t = _ct1.t + (1 - _ct1.t) * 0.25;
							float ft = c0.t + (c.t - c0.t) * pt;
							eval_point(i, ft, _ct1.x, _ct1.y);
							if(odb) eval_count_2++;
							//_ct1.x = c0.x + dx * _ct1.t;
							//_ct1.y = c0.y + dy * _ct1.t;
							
							float lx = c0.x + dx * pt;
							float ly = c0.y + dy * pt;
							
							//db.line(22, 22, _ct1.x,  _ct1.y, x, y, 3 * db_zf, 0x33ffff00, true, 1);
							if(odb) db.dot(22, 22, _ct1.x,  _ct1.y, 3*db_zf, 0xffffff00, 45, true, 1);
							if(odb) db.dot(22, 22, lx, ly, 3*db_zf, 0xffff00ff, 45, true, 1);
							if(odb) db.line(22, 22, lx, ly, x, y, 2*db_zf, 0x33ff00ff, true, 1);
							
							dx = x - lx;
							dy = y - ly;
							//normalize(dx, dy, dx, dy);
							float xx2, yy2;
							project(_ct1.x - lx, _ct1.y - ly, dx, dy, xx2, yy2);
							xx2 += lx;
							yy2 += ly;
							if(odb) db.dot(22, 22, xx2, yy2, 3*db_zf, 0xffff55ff, 45, true, 1);
							
							const float xxx = (xx2 - x) * (xx2 - x) + (yy2 - y) * (yy2 - y);
							const float xxx2 = (_ct1.x - x) * (_ct1.x - x) + (_ct1.y - y) * (_ct1.y - y);
							if(xxx < xxx2)
							{
								c_dist_a = xxx;
								//_ct1.x = xx2;
								//_ct1.y = yy2;
							}
							else
							{
								force_iteration = 0;
							}
							
							//const float arc_l = sqrt(dx * dx + dy * dy);
							//const float nx = dx / arc_l;
							//const float ny = dy / arc_l;
							//float x22 = _ct1.x;
							//float y22 = _ct1.y;
							//float dist_diff = 0;
							//const float line_l = sqrt(dx * pt * dx * pt + dy * pt * dy * pt);
							//
							//int ccc = 0;
							//puts(j+' -------------------------');
							//do
							//{
							//	float cx, cy;
							//	project(x22 - c0.x, y22 - c0.y, nx, ny, cx, cy);
							//	
							//	float c_l = sqrt(cx * cx + cy * cy);
							//	
							//	dist_diff = c_l - line_l;
							//	pt = clamp01(pt - dist_diff / c.length);
							//	ft = c0.t + (c.t - c0.t) * pt;
							//	eval_point(i, ft, x22, y22);
							//	if(odb) eval_count_2++;
							//	if(odb) db.dot(22, 24, x22, y22, 2*db_zf, 0xaaff0000, 45, true, 1);
							//	project(x22 - c0.x, y22 - c0.y, nx, ny, cx, cy);
							//	c_l = sqrt(cx * cx + cy * cy);
							//	puts(dist_diff, c_l - line_l);
							//	//if(ccc++ == 5)
							//	{
							//	//	puts('XX');
							//		break;
							//	}
							//}
							//while(abs(dist_diff) > threshold);
							
							@c = _ct1;
							c.t = ft;
						}
					}
				}
				
				if(j > 0 && arc_length_interpolation == CurveArcInterpolation::Eval)
				{
					CurveArc@ c0 = v.arcs[j - 1];
					float dx = c.x - c0.x;
					float dy = c.y - c0.y;
					float mt;
					float cm_dist;
					float t0 = c0.t;
					float t1 = c.t;
					
					int ccc = 0;
					if(dx != 0 || dy != 0)
					{
						float pt = ((x - c0.x) * dx + (y - c0.y) * dy) / (dx * dx + dy * dy);
						
						if(pt > 0 && pt < 1)
						{
							dx = x - c0.x;
							dy = y - c0.y;
							float c0_dist = dx * dx + dy * dy;
							
							dx = x - c.x;
							dy = y - c.y;
							float c1_dist = dx * dx + dy * dy;
							
							if(pt < 0)
							{
								cm_dist = c0_dist;
							}
							else if(pt > 1)
							{
								cm_dist = c1_dist;
							}
							else
							{
								mt = t0 + (t1 - t0) * pt;
								eval_point(i, mt, _ct1.x, _ct1.y);
								if(odb) eval_count_2++;
								
								dx = x - _ct1.x;
								dy = y - _ct1.y;
								cm_dist = dx * dx + dy * dy;
							}
							
							if(odb && pt > 0 && pt < 1) db.dot(22, 22, _ct1.x,  _ct1.y, 3*db_zf, 0xffffff00, 45, true, 1);
							
							do
							{
								//if(cm_dist <= c0_dist && cm_dist <= c0_dist)
								
								break;
							}
							while(true);
						}
					}
				}
				
				//if(c_dist_r == -1)
				{
					const float dx = x - c.x;
					const float dy = y - c.y;
					c_dist = dx * dx + dy * dy;
				}
				//else
				//{
				//	c_dist = c_dist_r;
				//}
				
				if((c_dist_a != -1 ? c_dist_a : c_dist) > dist_a)
					continue;
				
				if(@c == @_ct1)
				{
					_ct2 = _ct1;
					@c = _ct2;
				}
				
				dist = c_dist;
				dist_a = c_dist_a != -1 ? c_dist_a : c_dist;
				segment_index = i;
				arc_index = j;
				@clostest_arc = c;
			}
		}
		
		if(segment_index == -1)
			return false;
		
		// -- Step 2. Using the closest arc segment and the two surrounding points, do a binary search to find progressively closer
		// points until some threshold is reached.
		
		const bool is_interpolated = @clostest_arc == @_ct2;
		
		CurveVertex@ v = vertices[segment_index];
		px = clostest_arc.x;
		py = clostest_arc.y;
		t = segment_index + clostest_arc.t;
		
		const int si1 = arc_index > 0 || is_interpolated ? segment_index
			: segment_index > 0 ? segment_index - 1
			: segment_index;
		const int si2 = arc_index < v.arc_count - 1 || is_interpolated ? segment_index
			: closed || segment_index < end - 1 ? segment_index + 1
			: segment_index;
		
		const int ai1 = arc_index > 0 ? arc_index - 1
			: segment_index > 0 ? 1
			: arc_index;
		const int ai2 = arc_index < v.arc_count - 1 ? arc_index + 1
			: segment_index < end - 1 ? 1
			: arc_index;
		int di = 0;
		//if(odb) db.print(si1+'.'+ai1+'  '+segment_index+'.'+arc_index+'  '+si2+'.'+ai2, 0xffffffff, di++, 1);
		
		CurveArc@ c1 = arc_index > 0 ? v.arcs[arc_index - 1]
			: segment_index > 0 ? vertices[segment_index - 1].arc_from_end(1)
			: clostest_arc;
		CurveArc@ c2 = is_interpolated ? v.arcs[arc_index]
			: arc_index < v.arc_count - 1 ? v.arcs[arc_index + 1]
			: closed || segment_index < end - 1 ? vert(segment_index, 1).arc_from_start(1)
			: clostest_arc;
		float t1 = si1 + c1.t;
		float t2 = si2 + c2.t;
		float p1x = c1.x;
		float p1y = c1.y;
		float p2x = c2.x;
		float p2y = c2.y;
		if(odb) db.dot(22, 22, p1x,  p1y, 3*db_zf, 0xffff0000, 45, true, 1);
		if(odb) db.dot(22, 22, p2x,  p2y, 3*db_zf, 0xff00ff00, 45, true, 1);
		if(odb) db.print(str(x,y), 0xffff0000, di++, 1);
		if(odb) db.print(' t1 - '+str(t1,3)+':'+str(p1x,3), 0xffffffff, di++, 1);
		if(odb) db.print(' tm - '+str(t,3)+':'+str(px,3), 0xffffffff, di++, 1);
		if(odb) db.print(' t2 - '+str(t2,3)+':'+str(p2x,3), 0xffffffff, di++, 1);
		if(odb) db.print('        '+str(sqrt((p2x - p1x) * (p2x - p1x) + (p2y - p1y) * (p2y - p1y)),3), 0xffffffff, di++, 1);
		
		//if(dist_r != -1)
		//	dist = dist_r;
		
		threshold *= threshold;
		// Interpolating the initial guess usually makes it more acurate.
		// Making the bounds tighter initially and slowly increasing back to 0.5 seems to save on iterations and reach the threshold somewhat faster.
		float factor = tf && arc_length_interpolation >= CurveArcInterpolation::Linear ? 0.15 : 0.5;
		//factor = 0.5;
		
		do
		{
			float p1mx, p1my;
			float p2mx, p2my;
			
			// TODO: ?? Line project and interpolate t values instead of just using 0.5
			// TODO: ?? Bias towards initial guess? If we assume it's more likely to be closer to this point then this could reduce a few steps.
			
			// Left side.
			const float t1m = t + (t1 - t) * factor;
			const int i1 = (int(t1m) % vertex_count + vertex_count) % vertex_count;
			eval_point(i1, fraction(t1m), p1mx, p1my);
			if(odb) eval_count++;
			const float dist1m = (p1mx - x) * (p1mx - x) + (p1my - y) * (p1my - y);
			
			// Right side.
			const float t2m = t + (t2 - t) * factor;
			const int i2 = (int(t2m) % vertex_count + vertex_count) % vertex_count;
			eval_point(i2, fraction(t2m), p2mx, p2my);
			if(odb) eval_count++;
			const float dist2m = (p2mx - x) * (p2mx - x) + (p2my - y) * (p2my - y);
			
			//if(odb) db.print(' > -1 : '+i1+'/'+str(fraction(t1m), 3)+':'+str(p1mx,3)+' '+str(sqrt(dist1m),3), 0xffffffff, di++, 1);
			//if(odb) db.print('    m : '+str(t, 3)+':'+str(px,3)+' '+str(sqrt(dist),3), 0xffffffff, di++, 1);
			//if(odb) db.print('    1 : '+i2+'/'+str(fraction(t2m),3)+':'+str(p2mx,3)+' '+str(sqrt(dist2m),3), 0xffffffff, di++, 1);
			
			// Mid point is closest.
			if(dist <= dist1m && dist <= dist2m)
			{
				t1 = t1m;
				p1x = p1mx;
				p1y = p1my;
				t2 = t2m;
				p2x = p2mx;
				p2y = p2my;
				//if(odb) db.print('        >  m', 0xffffffff, di++, 1);
			}
			// Left point is closest.
			else if(dist1m < dist2m)
			{
				t2 = t;
				p2x = px;
				p2y = py;
				t = t1m;
				px = p1mx;
				py = p1my;
				dist = dist1m;
				//if(odb) db.print('        > -1', 0xffffffff, di++, 1);
			}
			// Right point is closest.
			else
			{
				t1 = t;
				p1x = px;
				p1y = py;
				t = t2m;
				px = p2mx;
				py = p2my;
				dist = dist2m;
				//if(odb) db.print('        >  1', 0xffffffff, di++, 1);
			}
			
			if(icap++ > 50 || eval_count > 150)
			{
				puts(' ????? '+str(p1x, p1y), str(p2x, p2y)+'  '+closeTo(t1, t2)+'  ' + str(t1, 20)+' '+str(t2, 20));
				break;
			}
			//if(odb) db.print('        '+str(p1x)+' '+str(p2x)+' '+str(sqrt((p2x - p1x) * (p2x - p1x) + (p2y - p1y) * (p2y - p1y)),3), 0xffffff00, di++, 1);
			
			//factor = 0.5;
			factor = factor + (0.5 - factor) * 0.25;
		}
		while(force_iteration-- > 0 || (p2x - p1x) * (p2x - p1x) + (p2y - p1y) * (p2y - p1y) > threshold && !closeTo(t1, t2));
		
		// TODO: Option to interpolate between closest points to produce a smoother result, which may not lie exactly on the curve.
		
		if(max_distance > 0 && (x - px) * (x - px) + (y - py) * (y - py) < max_distance * max_distance)
			return false;
		
		if(interpolate_result)
		{
			const float dx = p2x - p1x;
			const float dy = p2y - p1y;
			
			if(dx != 0 || dy != 0)
			{
				const float it = clamp01(((x - p1x) * dx + (y - p1y) * dy) / (dx * dx + dy * dy));
				t = t1 + (t2 - t1) * it;
				segment_index = (int(t) % vertex_count + vertex_count) % vertex_count;
				t = fraction(t);
				eval_point(segment_index, t, px, py);
				if(odb) eval_count++;
			}
		}
		else
		{
			t = fraction(t);
		}
		
		if(odb) db.print(' === '+str(t, 3), 0xffffffff, di++, 1);
		
		//db.print(eval_count+'', 0xff00ffff, -1, 1, false);
		
		return true;
	}
	
	/** Returns the vertices/control points for the segment at `i` based whether the curve is open/closed, etc. */
	void get_segment_catmull_rom(const int i, CurveControlPoint@ &out p1, CurveVertex@ &out p2, CurveVertex@ &out p3, CurveControlPoint@ &out p4)
	{
		@p2 = @vertices[i];
		@p3 = vert(i, 1);
		
		@p1 = p2.type != Square
			? closed || i > 0
				? vert(i, -1)
				: _end_controls != Manual
					? this.p0.extrapolate(p2, p3,
						_end_controls == CurveEndControl::AutomaticAngle && vertex_count >= 3 ? @vertices[2] : null)
					: check_control_point_start()
			: p2;
		@p4 = p3.type != Square
			? closed || i < vertex_count - 2
				? vert(i, 2)
				: _end_controls != Manual
					? this.p3.extrapolate(p3, p2,
						_end_controls == CurveEndControl::AutomaticAngle && vertex_count >= 3 ? @vertices[vertex_count - 3] : null)
					: check_control_point_end()
			: p3;
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
	
	/** Returns the vertex at `i + offset` wrapping around when < 0 or > vertex_count. */
	CurveVertex@ vert(const int i, const int offset=0)
	{
		return vertices[((i + offset) % vertex_count + vertex_count) % vertex_count];
	}
	
	// -- Bounding box methods --
	
	private void calc_bounding_box_linear()
	{
		const int end = closed ? vertex_count : vertex_count - 1;
		for(int i = 0; i < end; i++)
		{
			CurveVertex@ p1 = vertices[i];
			CurveVertex@ p2 = vert(i, 1);
			
			p1.x1 = p1.x < p2.x ? p1.x : p2.x;
			p1.y1 = p1.y < p2.y ? p1.y : p2.y;
			p1.x2 = p1.x > p2.x ? p1.x : p2.x;
			p1.y2 = p1.y > p2.y ? p1.y : p2.y;
			
			if(p1.x1 < x1) x1 = p1.x1;
			if(p1.y1 < y1) y1 = p1.y1;
			if(p1.x2 > x2) x2 = p1.x2;
			if(p1.y2 > y2) y2 = p1.y2;
		}
	}
	
	private void calc_bounding_box_catmull_rom()
	{
		const int end = closed ? vertex_count : vertex_count - 1;
		for(int i = 0; i < end; i++)
		{
			CurveVertex@ p2, p3;
			CurveControlPoint@ p1, p4;
			get_segment_catmull_rom(i, p1, p2, p3, p4);
			
			float bp1x, bp1y, bp2x, bp2y, bp3x, bp3y, bp4x, bp4y;
			CatmullRom::to_cubic_bezier(
				p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y, tension * p2.tension,
				bp1x, bp1y, bp2x, bp2y, bp3x, bp3y, bp4x, bp4y);
			bp2x += p2.x;
			bp2y += p2.y;
			bp3x += p3.x;
			bp3y += p3.y;
			
			CubicBezier::bounding_box(
				bp1x, bp1y, bp2x, bp2y, bp3x, bp3y, bp4x, bp4y,
				p2.x1, p2.y1, p2.x2, p2.y2);
			
			if(p2.x1 < x1) x1 = p2.x1;
			if(p2.y1 < y1) y1 = p2.y1;
			if(p2.x2 > x2) x2 = p2.x2;
			if(p2.y2 > y2) y2 = p2.y2;
		}
	}
	
	private void calc_bounding_box_quadratic_bezier()
	{
		const int end = closed ? vertex_count : vertex_count - 1;
		for(int i = 0; i < end; i++)
		{
			CurveVertex@ p1 = @vertices[i];
			const CurveVertex@ p2 = vert(i, 1);
			const CurveControlPoint@ cp = p1.quad_control_point;
			
			float sx1, sy1, sx2, sy2;
			
			if(p1.weight == 1 && cp.weight == 1 && p2.weight == 1)
			{
				QuadraticBezier::bounding_box(
					p1.x, p1.y, cp.x, cp.y, p2.x, p2.y,
					p1.x1, p1.y1, p1.x2, p1.y2);
			}
			else
			{
				QuadraticBezier::bounding_box(
					p1.x, p1.y, cp.x, cp.y, p2.x, p2.y,
					p1.weight, cp.weight, p2.weight,
					p1.x1, p1.y1, p1.x2, p1.y2);
			}
			
			if(p1.x1 < x1) x1 = p1.x1;
			if(p1.y1 < y1) y1 = p1.y1;
			if(p1.x2 > x2) x2 = p1.x2;
			if(p1.y2 > y2) y2 = p1.y2;
		}
	}
	
	private void calc_bounding_box_cubic_bezier(const int samples=6, const float padding=0.5)
	{
		const int end = closed ? vertex_count : vertex_count - 1;
		for(int i = 0; i < end; i++)
		{
			CurveVertex@ p1 = @vertices[i];
			const CurveVertex@ p2 = vert(i, 1);
			const CurveControlPoint@ cp1 = p1.cubic_control_point_2;
			const CurveControlPoint@ cp2 = p2.cubic_control_point_1;
			
			float sx1, sy1, sx2, sy2;
			
			if(p1.weight == 1 && cp1.weight == 1 &&  cp2.weight == 1 && p2.weight == 1)
			{
				CubicBezier::bounding_box(
					p1.x, p1.y, p1.x + cp1.x, p1.y + cp1.y,
					p2.x + cp2.x, p2.y + cp2.y, p2.x, p2.y,
					p1.x1, p1.y1, p1.x2, p1.y2);
			}
			else
			{
				CubicBezier::bounding_box(
					p1.x, p1.y, p1.x + cp1.x, p1.y + cp1.y,
					p2.x + cp2.x, p2.y + cp2.y, p2.x, p2.y,
					p1.weight, cp1.weight, cp2.weight, p2.weight,
					p1.x1, p1.y1, p1.x2, p1.y2,
					samples, padding);
			}
			
			if(p1.x1 < x1) x1 = p1.x1;
			if(p1.y1 < y1) y1 = p1.y1;
			if(p1.x2 > x2) x2 = p1.x2;
			if(p1.y2 > y2) y2 = p1.y2;
		}
	}
	
	private void calc_bounding_box_b_spline()
	{
		b_spline.bounding_box_simple(
			vertex_count, _b_spline_degree, _closed,
			x1, y1, x2, y2);
	}
	
	// -- Util --
	
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
			invalidated_b_spline_knots = true;
		}
		
		invalidated_b_spline_vertices = true;
	}
	
	private void calc_segment_t(const int segment, const float t, float & out ts, int &out i)
	{
		const int max_i = closed ? vertex_count - 1 : vertex_count - 2;
		
		if(segment < 0)
		{
			const float tt = t * (max_i + 1);
			i = int(tt);
			i = i <= max_i ? i : max_i;
			ts = i <= max_i ? tt % 1 : 1;
		}
		else if(segment > max_i)
		{
			i = max_i;
			ts = 1;
		}
		else
		{
			i = segment;
			ts = t < 1 ? t : t > 0 ? t : 0;
		}
	}
	
}
