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
	
	// TODO: Calculate individual segment bounding boxes.
	// TODO: Debug draw arc lengths
	// TODO: 	- Display segment lengths
	// TODO: Tweak bspline so vertex and segment align better for closed bsplines.
	// TODO: `invalidate` and `validate` seem kind of redundant?
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
		
		length = Curve::calculate_arc_lengths(
			@vertices, vertex_count, _closed,
			eval_func_def, _type != Linear ? 6 : 1,
			_type != Linear ? 3 * DEG2RAD : 0, 5);
		
		invalidated = false;
	}
	
	/** Call to update/calcualte some simple initial positions for new control points.
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
			case Linear:
				eval_linear(segment, t, x, y, normal_x, normal_y, normalise);
				break;
			case QuadraticBezier:
				eval_quadratic_bezier(segment, t, x, y, normal_x, normal_y, normalise);
				break;
			case CubicBezier:
				eval_cubic_bezier(segment, t, x, y, normal_x, normal_y, normalise);
				break;
			case CatmullRom:
				eval_catmull_rom(segment, t, x, y, normal_x, normal_y, normalise);
				break;
			case BSpline:
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
			case Linear:
				eval_linear_point(segment, t, x, y);
				break;
			case QuadraticBezier:
				eval_quadratic_bezier_point(segment, t, x, y);
				break;
			case CubicBezier:
				eval_cubic_bezier_point(segment, t, x, y);
				break;
			case CatmullRom:
				eval_catmull_rom_point(segment, t, x, y);
				break;
			case BSpline:
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
			case Linear:
				eval_linear_normal(segment, t, normal_x, normal_y, normalise);
				break;
			case QuadraticBezier:
				eval_quadratic_bezier_normal(segment, t, normal_x, normal_y, normalise);
				break;
			case CubicBezier:
				eval_cubic_bezier_normal(segment, t, normal_x, normal_y, normalise);
				break;
			case CatmullRom:
				eval_catmull_rom_normal(segment, t, normal_x, normal_y, normalise);
				break;
			case BSpline:
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
		for(int i = 0; i < vertex_count; i++)
		{
			CurveVertex@ p = vertices[i];
			
			if(p.x < x1) x1 = p.x;
			if(p.y < y1) y1 = p.y;
			if(p.x > x2) x2 = p.x;
			if(p.y > y2) y2 = p.y;
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
			
			float sx1, sy1, sx2, sy2;
			CubicBezier::bounding_box(
				bp1x, bp1y, bp2x, bp2y, bp3x, bp3y, bp4x, bp4y,
				sx1, sy1, sx2, sy2);
			
			if(sx1 < x1) x1 = sx1;
			if(sy1 < y1) y1 = sy1;
			if(sx2 > x2) x2 = sx2;
			if(sy2 > y2) y2 = sy2;
		}
	}
	
	private void calc_bounding_box_quadratic_bezier()
	{
		const int end = closed ? vertex_count : vertex_count - 1;
		for(int i = 0; i < end; i++)
		{
			const CurveVertex@ p1 = @vertices[i];
			const CurveVertex@ p2 = vert(i, 1);
			const CurveControlPoint@ cp = p1.quad_control_point;
			
			float sx1, sy1, sx2, sy2;
			
			if(p1.weight == 1 && cp.weight == 1 && p2.weight == 1)
			{
				QuadraticBezier::bounding_box(
					p1.x, p1.y, cp.x, cp.y, p2.x, p2.y,
					sx1, sy1, sx2, sy2);
			}
			else
			{
				QuadraticBezier::bounding_box(
					p1.x, p1.y, cp.x, cp.y, p2.x, p2.y,
					p1.weight, cp.weight, p2.weight,
					sx1, sy1, sx2, sy2);
			}
			
			if(sx1 < x1) x1 = sx1;
			if(sy1 < y1) y1 = sy1;
			if(sx2 > x2) x2 = sx2;
			if(sy2 > y2) y2 = sy2;
		}
	}
	
	private void calc_bounding_box_cubic_bezier(const int samples=6, const float padding=0.5)
	{
		const int end = closed ? vertex_count : vertex_count - 1;
		for(int i = 0; i < end; i++)
		{
			const CurveVertex@ p1 = @vertices[i];
			const CurveVertex@ p2 = vert(i, 1);
			const CurveControlPoint@ cp1 = p1.cubic_control_point_2;
			const CurveControlPoint@ cp2 = p2.cubic_control_point_1;
			
			float sx1, sy1, sx2, sy2;
			
			if(p1.weight == 1 && cp1.weight == 1 &&  cp2.weight == 1 && p2.weight == 1)
			{
				CubicBezier::bounding_box(
					p1.x, p1.y, p1.x + cp1.x, p1.y + cp1.y,
					p2.x + cp2.x, p2.y + cp2.y, p2.x, p2.y,
					sx1, sy1, sx2, sy2);
			}
			else
			{
				CubicBezier::bounding_box(
					p1.x, p1.y, p1.x + cp1.x, p1.y + cp1.y,
					p2.x + cp2.x, p2.y + cp2.y, p2.x, p2.y,
					p1.weight, cp1.weight, cp2.weight, p2.weight,
					sx1, sy1, sx2, sy2,
					samples, padding);
			}
			
			if(sx1 < x1) x1 = sx1;
			if(sy1 < y1) y1 = sy1;
			if(sx2 > x2) x2 = sx2;
			if(sy2 > y2) y2 = sy2;
		}
	}
	
	private void calc_bounding_box_b_spline()
	{
		b_spline.bounding_box_simple(x1, y1, x2, y2);
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
		else
		{
			i = segment <= max_i ? segment : max_i;
			ts = t < 1 ? t : t > 0 ? t : 0;
		}
	}
	
}
