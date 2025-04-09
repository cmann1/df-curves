#include 'b_spline.cpp';
#include 'catmull_rom.cpp';
#include 'catmull_rom_to_cubic_bezier.cpp';
#include 'cubic.cpp';
#include 'cubic_rational.cpp';
#include 'cubic_bounding_box.cpp';
#include 'cubic_bounding_box_rational.cpp';
#include 'cubic_split.cpp';
#include 'cubic_split_rational.cpp';
#include 'quadratic.cpp';
#include 'quadratic_rational.cpp';
#include 'quadratic_bounding_box.cpp';
#include 'quadratic_bounding_box_rational.cpp';
#include 'quadratic_split.cpp';
#include 'quadratic_split_rational.cpp';

#include 'CurveVertex.cpp';
#include 'calculate_arc_lengths.cpp';
#include 'closest_point.cpp';

#include 'CurveControlPointDrag.cpp';
#include 'CurveDrag.cpp';
#include 'CurveDragType.cpp';
#include 'MultiCuveSubdivisionSettings.cpp';

/** A higher level wrapper designed for editing/manipulating different types of curves. */
class MultiCurve
{
	
	// TODO: Transformin (translate/scale/rotate) all points.
	// TODO: ? Multi select vertices.
	// TODO: 	Drag/trasnform
	// TODO: Copy entire MultiCurve to/from.
	
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
	
	/** Controls the base number of the pre-calculated subdivisions of this curve.
	  * These subdivisions are required for certain operations, e.g. approximating the curve length, or finding
	  * a closest point on the curve.
	  * The default values skew towards accuracy over performance.
	  * Changes to these values will only take effect when the curve is next validated.
	  * See `Curve::calculate_arc_lengths` for descriptions of these properties. */
	MultiCuveSubdivisionSettings subdivision_settings;
	
	/** The total (approximate) length of this curve. */
	float length;
	
	/** This curve's bounding box. */
	float x1, y1, x2, y2;
	
	// --
	
	/** One or more segments on this curve have been changed and need to be updated. */
	private bool invalidated = true;
	
	/** The b-spline weighted vertices requires regeneration after vertices are added/removed. */
	private bool invalidated_b_spline_vertices = true;
	
	/** The b-spline knot vector requires regeneration after vertices are added/removed. */
	private bool invalidated_b_spline_knots = true;
	
	/** Control points may not be initialised after changing curve type. */
	private bool invalidated_control_points = true;
	
	private BSpline@ b_spline;
	
	/** Temp points used when calculating automatic end control points. */
	private CurveVertex p0;
	private CurveVertex p3;
	
	private Curve::EvalFunc@ eval_func_def;
	private Curve::EvalPointFunc@ eval_point_func_def;
	
	// -- Editing/dragging stuff
	
	private CurveDrag drag_curve;
	private array<CurveControlPointDrag> drag_control_points(2);
	private int drag_control_points_count;
	
	MultiCurve()
	{
		@eval_func_def = Curve::EvalFunc(eval);
		@eval_point_func_def = Curve::EvalPointFunc(eval_point);
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
			
			invalidated = true;
			invalidated_b_spline_knots = true;
			invalidated_b_spline_vertices = true;
			invalidated_control_points = true;
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
			
			if(_closed)
			{
				CurveVertex@ v = vert(-1);
				if(@v != null)
				{
					v.invalidated = true;
				}
			}
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
	
	CurveVertex@ first_vertex
	{
		get { return vertex_count > 0 ? vertices[0] : null; }
	}
	
	CurveVertex@ last_vertex
	{
		get { return vertex_count > 0 ? vertices[vertex_count - 1] : null; }
	}
	
	const int segment_index_max
	{
		get const { return _closed ? vertex_count - 1 : vertex_count - 2; }
	}
	
	bool is_invalidated
	{
		get const { return invalidated; }
	}
	
	const bool is_end_control(CurveControlPoint@ p)
	{
		return @p == @control_point_start || @p == @control_point_end;
	}
	
	/** Returns the ratio/weighted calculated during the last `eval_**` call. */
	float b_spline_last_ratio
	{
		get const { return @b_spline != null ? b_spline.last_w : 1; }
	}
	
	// --
	
	/** Call after modifying this curve in any way, so that cached values such as lengths, bounding boxes, etc. can be recalculated.
	  * Certain operation such as adding vertices, changing the curve type, etc. will automatically invalidate the curve, but directly setting properties
	  * such as vertex position will require manually calling `invalidate`. */
	void invalidate()
	{
		if(vertex_count == 0)
			return;
		
		invalidated = true;
		invalidated_b_spline_vertices = true;
		
		const int end_index = segment_index_max;
		for(int i = 0; i <= end_index; i++)
		{
			vertices[i].invalidated = true;
		}
	}
	
	/** Invalidate a single, or range of vertices. Potentially invalidates surrounding vertices depending on the curve type. */
	void invalidate(const int start_index, const int end_index=-1)
	{
		if(vertex_count == 0)
			return;
		
		invalidated = true;
		invalidated_b_spline_vertices = true;
		
		int o1, o2;
		get_affected_vertex_offsets(o1, o2);
		
		const int i1 = min(start_index < 0 ? 0 : start_index, vertex_count - 1) + o1;
		const int i2 = min(end_index < 0 ? start_index : end_index, vertex_count - 1) + o2;
		for(int i = i1; i <= i2; i++)
		{
			if(!_closed && (i < 0 || i >= vertex_count))
				continue;
			
			vertices[(i % vertex_count + vertex_count) % vertex_count].invalidated = true;
		}
	}
	
	/** Invalidate a single vertix/curve segment.
	  * Can be used when modifying a property that only affects the given segment, e.g. tension, or control points.
	  * When modifying anything else, e.g. vertex position, use `invalidate` instead. */
	void invalidate(const int index, const bool segment)
	{
		if(vertex_count == 0)
			return;
		
		if(!segment)
		{
			invalidate(index);
			return;
		}
		
		if(index < 0 || index > segment_index_max)
			return;
		
		invalidated = true;
		invalidated_b_spline_vertices = true;
		vertices[index].invalidated = true;
	}
	
	/** Must be called after `invalidate` and any time the curve is modified in any way.
	  * Recalculates cached values such as the bounding box, curve length, etc. */
	void validate()
	{
		if(!invalidated)
			return;
		
		if(invalidated_control_points)
		{
			init_bezier_control_points();
			invalidated_control_points = false;
		}
		
		validate_b_spline();
		
		// -- Calculate arc lengths.
		
		length = Curve::calculate_arc_lengths(
			@vertices, vertex_count, _closed,
			eval_func_def, true, _type != Linear ? subdivision_settings.count : 1,
			_type != Linear ? subdivision_settings.angle_min * DEG2RAD : 0,
			subdivision_settings.max_stretch_factor, subdivision_settings.length_min,
			subdivision_settings.max_subdivisions,
			subdivision_settings.angle_max * DEG2RAD, subdivision_settings.length_max);
		
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
		
		// -- Finish
		
		invalidated = false;
		
		const int end = segment_index_max;
		for(int i = 0; i <= end; i++)
		{
			vertices[i].invalidated = false;
		}
	}
	
	private void validate_b_spline()
	{
		if(_type != CurveType::BSpline)
			return;
		
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
			CurveVertex@ p1 = vert(i);
			if(@p1 == null)
				continue;
			
			CurveControlPoint@ cp1 = p1.cubic_control_point_1;
			CurveControlPoint@ cp2 = p1.cubic_control_point_2;
			
			if(!force && !is_nan(cp1.x) && !is_nan(cp2.x))
				continue;
			
			const CurveVertex@ p0 = vert(i - 1);
			const CurveVertex@ p2 = vertex_count > 2 ? vert(i + 1) : vertices[i];
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
			CurveVertex@ p1 = @vert(i);
			if(@p1 == null)
				continue;
			CurveControlPoint@ cp = p1.quad_control_point;
			
			if(!force && !is_nan(cp.x))
				continue;
			
			const CurveVertex@ p0 = vert(i - 1);
			const CurveVertex@ p2 = vertex_count > 2 ? vert(i + 1) : vertices[i];
			const float tx = p2.x - p0.x;
			const float ty = p2.y - p0.y;
			
			if(force || is_nan(cp.x))
			{
				cp.x = tx * 0.5;
				cp.y = ty * 0.5;
			}
		}
	}
	
	// -- Eval methods --
	
	/** Calculate the position and normal at the given segment and t value.
	 * To return evenly spaced points along the curve, use the mapping methods to convert a distance to a t value.
	 * @param segment The index of the segment between 0 and `vertex_count` for closed curves, and `vertex_count` for open.
	 *                Passing a negative values will instead calculate the segment index automatically, and `t` will be considered
	 *                an absolute value with 0 being the first vertex of the curve, and 1 being the last.
	 * @param t The factor between 0 and 1 within `segment`, or the entire curve if `segment` is negative. */
	void eval(const int segment, const float t, float &out x, float &out y, float &out normal_x, float &out normal_y)
	{
		switch(vertex_count)
		{
			case 0:
			{
				x = 0;
				y = 0;
				normal_x = 1;
				normal_y = 0;
				return;
			}
			case 1:
			{
				const CurveVertex@ p0 = @vertices[0];
				x = p0.x;
				y = p0.y;
				normal_x = 1;
				normal_y = 0;
				return;
			}
		}
		
		switch(_type)
		{
			case CurveType::Linear:
				eval_linear(segment, t, x, y, normal_x, normal_y);
				break;
			case CurveType::QuadraticBezier:
				eval_quadratic_bezier(segment, t, x, y, normal_x, normal_y);
				break;
			case CurveType::CubicBezier:
				eval_cubic_bezier(segment, t, x, y, normal_x, normal_y);
				break;
			case CurveType::CatmullRom:
				eval_catmull_rom(segment, t, x, y, normal_x, normal_y);
				break;
			case CurveType::BSpline:
				eval_b_spline(segment, t, x, y, normal_x, normal_y);
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
	void eval_normal(const int segment, const float t, float &out normal_x, float &out normal_y)
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
				eval_linear_normal(segment, t, normal_x, normal_y);
				break;
			case CurveType::QuadraticBezier:
				eval_quadratic_bezier_normal(segment, t, normal_x, normal_y);
				break;
			case CurveType::CubicBezier:
				eval_cubic_bezier_normal(segment, t, normal_x, normal_y);
				break;
			case CurveType::CatmullRom:
				eval_catmull_rom_normal(segment, t, normal_x, normal_y);
				break;
			case CurveType::BSpline:
				eval_b_spline_normal(segment, t, normal_x, normal_y);
				break;
			default:
				normal_x = 1;
				normal_y = 0;
				break;
		}
	}
	
	
	/** Returns the ratio/weight at the given t value. */
	float eval_ratio(const int segment, const float t)
	{
		switch(vertex_count)
		{
			case 0:
				return 1;
			case 1:
				return vertices[0].weight;
		}
		
		switch(_type)
		{
			case CurveType::Linear:
				return eval_linear_ratio(segment, t);
			case CurveType::QuadraticBezier:
				return eval_quadratic_bezier_ratio(segment, t);
			case CurveType::CubicBezier:
				return eval_cubic_bezier_ratio(segment, t);
			case CurveType::CatmullRom:
				return eval_catmull_rom_ratio(segment, t);
			case CurveType::BSpline:
				return eval_b_spline_ratio(segment, t);
		}
		
		return 1;
	}
	
	void eval_linear(
		const int segment, const float t, float &out x, float &out y, float &out normal_x, float &out normal_y)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		// Get vertices.
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p2 = vert(i + 1);
		
		// Calculate the point.
		const float dx = p2.x - p1.x;
		const float dy = p2.y - p1.y;
		
		x = p1.x + dx * ti;
		y = p1.y + dy * ti;
		
		// Calculate the normal vector.
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
	
	void eval_linear_point(const int segment, const float t, float &out x, float &out y)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		// Get vertices.
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p2 = vert(i + 1);
		
		// Calculate the point.
		x = p1.x + (p2.x - p1.x) * ti;
		y = p1.y + (p2.y - p1.y) * ti;
	}
	
	void eval_linear_normal(const int segment, const float t, float &out normal_x, float &out normal_y)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		// Get vertices.
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p2 = vert(i + 1);
		
		// Calculate the normal vector.
		normal_y = -(p2.x - p1.x);
		normal_x = p2.y - p1.y;
		
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
	
	float eval_linear_ratio(const int segment, const float t)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		// Get vertices.
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p2 = vert(i + 1);
		
		return p1.weight + (p2.weight - p1.weight) * ti;
	}
	
	void eval_catmull_rom(
		const int segment, const float t, float &out x, float &out y, float &out normal_x, float &out normal_y)
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
			ti, x, y, normal_x, normal_y);
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
	
	void eval_catmull_rom_normal(const int segment, const float t, float &out normal_x, float &out normal_y)
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
			ti, normal_x, normal_y);
	}
	
	float eval_catmull_rom_ratio(const int segment, const float t)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		CurveVertex@ p2, p3;
		CurveControlPoint@ p1, p4;
		get_segment_catmull_rom(i, p1, p2, p3, p4);
		
		return p2.weight + (p3.weight - p2.weight) * ti;
	}
	
	void eval_quadratic_bezier(const int segment, const float t, float &out x, float &out y, float &out normal_x, float &out normal_y)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		// Get vertices.
		const CurveVertex@ p1 = @vertices[i];
		const CurveControlPoint@ p2 = p1.quad_control_point;
		const CurveVertex@ p3 = vert(i + 1);
		
		// Linear fallback.
		if(p2.type == Square)
		{
			eval_linear(segment, t, x, y, normal_x, normal_y);
			return;
		}
		
		// Non-rational.
		if(p1.weight == p2.weight && p2.weight == p3.weight)
		{
			QuadraticBezier::eval(
				p1.x, p1.y, p1.x + p2.x, p1.y + p2.y, p3.x, p3.y,
				ti, x, y, normal_x, normal_y);
		}
		// Rational.
		else
		{
			QuadraticBezier::eval(
				p1.x, p1.y, p1.x + p2.x, p1.y + p2.y, p3.x, p3.y,
				p1.weight, p2.weight, p3.weight,
				ti, x, y, normal_x, normal_y);
		}
	}
	
	void eval_quadratic_bezier_point(const int segment, const float t, float &out x, float &out y)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		// Get vertices.
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p3 = vert(i + 1);
		const CurveControlPoint@ p2 = p1.quad_control_point;
		
		// Linear fallback.
		if(p2.type == Square)
		{
			eval_linear_point(segment, t, x, y);
			return;
		}
		
		// Non-rational.
		if(p1.weight == p2.weight && p2.weight == p3.weight)
		{
			QuadraticBezier::eval_point(
				p1.x, p1.y, p1.x + p2.x, p1.y + p2.y, p3.x, p3.y,
				ti, x, y);
		}
		// Rational.
		else
		{
			QuadraticBezier::eval_point(
				p1.x, p1.y, p1.x + p2.x, p1.y + p2.y, p3.x, p3.y,
				p1.weight, p2.weight, p3.weight,
				ti, x, y);
		}
	}
	
	void eval_quadratic_bezier_normal(const int segment, const float t, float &out normal_x, float &out normal_y)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		// Get vertices.
		const CurveVertex@ p1 = @vertices[i];
		const CurveControlPoint@ p2 = p1.quad_control_point;
		const CurveVertex@ p3 = vert(i + 1);
		
		// Linear fallback.
		if(p2.type == Square)
		{
			eval_linear_normal(segment, t, normal_x, normal_y);
			return;
		}
		
		// Non-rational.
		if(p1.weight == p2.weight && p2.weight == p3.weight)
		{
			QuadraticBezier::eval_normal(
				p1.x, p1.y, p1.x + p2.x, p1.y + p2.y, p3.x, p3.y,
				ti, normal_x, normal_y);
		}
		// Rational.
		else
		{
			QuadraticBezier::eval_normal(
				p1.x, p1.y, p1.x + p2.x, p1.y + p2.y, p3.x, p3.y,
				p1.weight, p2.weight, p3.weight,
				ti, normal_x, normal_y);
		}
	}
	
	float eval_quadratic_bezier_ratio(const int segment, const float t)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		// Get vertices.
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p3 = vert(i + 1);
		const CurveControlPoint@ p2 = p1.quad_control_point;
		
		// Linear fallback.
		if(p2.type == Square)
			return eval_linear_ratio(segment, t);
		
		// Non-rational.
		if(p1.weight == p2.weight && p2.weight == p3.weight)
			return p1.weight;
		
		// Rational.
		return QuadraticBezier::eval_ratio(
			p1.x, p1.y, p1.x + p2.x, p1.y + p2.y, p3.x, p3.y,
			p1.weight, p2.weight, p3.weight,
			ti);
	}
	
	void eval_cubic_bezier(const int segment, const float t, float &out x, float &out y, float &out normal_x, float &out normal_y)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p4 = vert(i + 1);
		const CurveControlPoint@ p2 = p1.cubic_control_point_2;
		const CurveControlPoint@ p3 = p4.cubic_control_point_1;
		
		// Linear fallback.
		if(p2.type == Square && p3.type == Square)
		{
			eval_linear(segment, t, x, y, normal_x, normal_y);
			return;
		}
		
		// Quadratic fallback.
		if(p2.type == Square || p3.type == Square)
		{
			const CurveControlPoint@ qp2 = p2.type == Square ? p4.cubic_control_point_1 : p1.cubic_control_point_2;
			const CurveControlPoint@ p0 = p2.type == Square ? p4 : p1;
			
			// Non-rational.
			if(p1.weight == qp2.weight && qp2.weight == p4.weight)
			{
				QuadraticBezier::eval(
					p1.x, p1.y, p0.x + qp2.x, p0.y + qp2.y, p4.x, p4.y,
					ti, x, y, normal_x, normal_y);
			}
			// Rational.
			else
			{
				QuadraticBezier::eval(
					p1.x, p1.y, p0.x + qp2.x, p0.y + qp2.y, p4.x, p4.y,
					p1.weight, qp2.weight, p4.weight,
					ti, x, y, normal_x, normal_y);
			}
			return;
		}
		
		// Non-rational.
		if(p1.weight == p2.weight && p2.weight == p3.weight && p3.weight == p4.weight)
		{
			CubicBezier::eval(
				p1.x, p1.y, p1.x + p2.x, p1.y + p2.y, p4.x + p3.x, p4.y + p3.y, p4.x, p4.y,
				ti, x, y, normal_x, normal_y);
		}
		// Rational.
		else
		{
			CubicBezier::eval(
				p1.x, p1.y, p1.x + p2.x, p1.y + p2.y, p4.x + p3.x, p4.y + p3.y, p4.x, p4.y,
				p1.weight, p2.weight, p3.weight, p4.weight,
				ti, x, y, normal_x, normal_y);
		}
	}
	
	void eval_cubic_bezier_point(const int segment, const float t, float &out x, float &out y)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p4 = vert(i + 1);
		const CurveControlPoint@ p2 = p1.cubic_control_point_2;
		const CurveControlPoint@ p3 = p4.cubic_control_point_1;
		
		// Linear fallback.
		if(p2.type == Square && p3.type == Square)
		{
			eval_linear_point(segment, t, x, y);
			return;
		}
		
		// Quadratic fallback.
		if(p2.type == Square || p3.type == Square)
		{
			const CurveControlPoint@ qp2 = p2.type == Square ? p4.cubic_control_point_1 : p1.cubic_control_point_2;
			const CurveControlPoint@ p0 = p2.type == Square ? p4 : p1;
			
			// Non-rational.
			if(p1.weight == qp2.weight && qp2.weight == p4.weight)
			{
				QuadraticBezier::eval_point(
					p1.x, p1.y, p0.x + qp2.x, p0.y + qp2.y, p4.x, p4.y,
					ti, x, y);
			}
			// Rational.
			else
			{
				QuadraticBezier::eval_point(
					p1.x, p1.y, p0.x + qp2.x, p0.y + qp2.y, p4.x, p4.y,
					p1.weight, qp2.weight, p4.weight,
					ti, x, y);
			}
			return;
		}
		
		// Non-rational.
		if(p1.weight == p2.weight && p2.weight == p3.weight && p3.weight == p4.weight)
		{
			CubicBezier::eval_point(
				p1.x, p1.y, p1.x + p2.x, p1.y + p2.y, p4.x + p3.x, p4.y + p3.y, p4.x, p4.y,
				ti, x, y);
		}
		// Rational.
		else
		{
			CubicBezier::eval_point(
				p1.x, p1.y, p1.x + p2.x, p1.y + p2.y, p4.x + p3.x, p4.y + p3.y, p4.x, p4.y,
				p1.weight, p2.weight, p3.weight, p4.weight,
				ti, x, y);
		}
	}
	
	void eval_cubic_bezier_normal(const int segment, const float t, float &out normal_x, float &out normal_y)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p4 = vert(i + 1);
		const CurveControlPoint@ p2 = p1.cubic_control_point_2;
		const CurveControlPoint@ p3 = p4.cubic_control_point_1;
		
		// Linear fallback.
		if(p2.type == Square && p3.type == Square)
		{
			eval_linear_normal(segment, t, normal_x, normal_y);
			return;
		}
		
		// Quadratic fallback.
		if(p2.type == Square || p3.type == Square)
		{
			const CurveControlPoint@ qp2 = p2.type == Square ? p4.cubic_control_point_1 : p1.cubic_control_point_2;
			const CurveControlPoint@ p0 = p2.type == Square ? p4 : p1;
			
			// Non-rational.
			if(p1.weight == p2.weight && p2.weight == p4.weight)
			{
				QuadraticBezier::eval_normal(
					p1.x, p1.y, p0.x + p2.x, p0.y + p2.y, p4.x, p4.y,
					ti, normal_x, normal_y);
			}
			// Rational.
			else
			{
				QuadraticBezier::eval_normal(
					p1.x, p1.y, p0.x + p2.x, p0.y + p2.y, p4.x, p4.y,
					p1.weight, p2.weight, p4.weight,
					ti, normal_x, normal_y);
			}
			return;
		}
		
		// Non-rational.
		if(p1.weight == p2.weight && p2.weight == p3.weight && p3.weight == p4.weight)
		{
			CubicBezier::eval_normal(
				p1.x, p1.y, p1.x + p2.x, p1.y + p2.y, p4.x + p3.x, p4.y + p3.y, p4.x, p4.y,
				ti, normal_x, normal_y);
		}
		// Rational.
		else
		{
			CubicBezier::eval_normal(
				p1.x, p1.y, p1.x + p2.x, p1.y + p2.y, p4.x + p3.x, p4.y + p3.y, p4.x, p4.y,
				p1.weight, p2.weight, p3.weight, p4.weight,
				ti, normal_x, normal_y);
		}
	}
	
	float eval_cubic_bezier_ratio(const int segment, const float t)
	{
		int i;
		float ti;
		calc_segment_t(segment, t, ti, i);
		
		const CurveVertex@ p1 = @vertices[i];
		const CurveVertex@ p4 = vert(i + 1);
		const CurveControlPoint@ p2 = p1.cubic_control_point_2;
		const CurveControlPoint@ p3 = p4.cubic_control_point_1;
		
		// Linear fallback.
		if(p2.type == Square && p3.type == Square)
			return eval_linear_ratio(segment, t);
		
		// Quadratic fallback.
		if(p2.type == Square || p3.type == Square)
		{
			const CurveControlPoint@ qp2 = p2.type == Square ? @p4.cubic_control_point_1 : @p1.cubic_control_point_2;
			const CurveControlPoint@ p0 = p2.type == Square ? p4 : p1;
			
			// Non-rational.
			if(p1.weight == qp2.weight && qp2.weight == p4.weight)
				return p1.weight;
			
			// Rational.
			return QuadraticBezier::eval_ratio(
				p1.x, p1.y, p0.x + p0.x, p0.y + p0.y, p4.x, p4.y,
				p1.weight, qp2.weight, p4.weight,
				ti);
		}
		
		// Non-rational.
		if(p1.weight == p2.weight && p2.weight == p3.weight && p3.weight == p4.weight)
			return p1.weight;
		
		// Rational.
		return CubicBezier::eval_ratio(
			p1.x, p1.y, p1.x + p2.x, p1.y + p2.y, p4.x + p3.x, p4.y + p3.y, p4.x, p4.y,
			p1.weight, p2.weight, p3.weight, p4.weight,
			ti);
	}
	
	void eval_b_spline(
		const int segment, const float t, float &out x, float &out y, float &out normal_x, float &out normal_y)
	{
		if(b_spline_degree <= 1)
		{
			eval_linear(segment, t, x, y, normal_x, normal_y);
			return;
		}
		
		const float ta = calc_b_spline_t(segment, t);
		b_spline.eval(
			b_spline_degree, b_spline_clamped, closed, 
			ta, x, y, normal_x, normal_y);
	}
	
	void eval_b_spline_point(const int segment, const float t, float &out x, float &out y)
	{
		if(b_spline_degree <= 1)
		{
			eval_linear_point(segment, t, x, y);
			return;
		}
		
		const float ta = segment >= 0
			? (segment + (t > 0 ? t : t < 1 ? t : 1)) / (_closed ? vertex_count : vertex_count - 1)
			: t;
		b_spline.eval_point(
			b_spline_degree, b_spline_clamped, closed, 
			ta, x, y);
	}
	
	void eval_b_spline_normal(const int segment, const float t, float &out normal_x, float &out normal_y)
	{
		if(b_spline_degree <= 1)
		{
			eval_linear_normal(segment, t, normal_x, normal_y);
			return;
		}
		
		const float ta = segment >= 0
			? (segment + (t > 0 ? t : t < 1 ? t : 1)) / (_closed ? vertex_count : vertex_count - 1)
			: t;
		b_spline.eval_normal(
			b_spline_degree, b_spline_clamped, closed, 
			ta, normal_x, normal_y);
	}
	
	float eval_b_spline_ratio(const int segment, const float t)
	{
		if(b_spline_degree <= 1)
			return eval_linear_ratio(segment, t);
		
		const float ta = calc_b_spline_t(segment, t);
		return b_spline.eval_ratio(
			b_spline_degree, b_spline_clamped, closed, 
			ta);
	}
	
	// --
	
	/** See `Curve::closest_point`. */
	bool closest_point(
		const float x, const float y, int &out segment_index, float &out t, float &out px, float &out py,
		const float max_distance=0, float threshold=1,
		const bool arc_length_interpolation=true,
		const bool adjust_initial_binary_factor=true,
		const bool interpolate_result=true)
	{
		return Curve::closest_point(
			vertices, vertex_count, closed,
			eval_point_func_def,
			x, y, segment_index, t, px, py,
			max_distance, threshold,
			arc_length_interpolation,
			adjust_initial_binary_factor,
			interpolate_result,
			x1, y1, x2, y2);
	}
	
	// -- Modification methods --
	
	void clear()
	{
		vertices.resize(0);
		vertex_count = 0;
		
		control_point_start.type = None;
		control_point_end.type = None;
		
		invalidated = true;
		invalidated_b_spline_knots = true;
		invalidated_b_spline_vertices = true;
		invalidated_control_points = true;
	}
	
	CurveVertex@ add_vertex(const float x, const float y)
	{
		vertices.resize(vertices.length + 1);
		CurveVertex@ v = vertices[vertex_count++];
		v.x = x;
		v.y = y;
		
		invalidated = true;
		invalidated_b_spline_knots = true;
		invalidated_b_spline_vertices = true;
		invalidated_control_points = true;
		
		return v;
	}
	
	bool remove_vertex(CurveVertex@ vertex)
	{
		const int index = vertices.findByRef(vertex);
		return index != -1 && remove_vertex(index);
	}
	
	bool remove_vertex(const int index)
	{
		const int i = (index % vertex_count + vertex_count) % vertex_count;
		vertices.removeAt(i);
		vertex_count--;
		
		invalidate(i);
		invalidated_b_spline_knots = true;
		invalidated_b_spline_vertices = true;
		
		return true;
	}
	
	// -- Insert vertex --
	//{
	
	/** Inserts a vertex in the given segment index, at the given position. */
	int insert_vertex(const int segment, const float x, const float y)
	{
		int index;
		CurveVertex@ p;
		
		if(segment >= vertex_count - 1)
		{
			vertices.resize(vertices.length + 1);
			index = vertex_count++;
			@p = vertices[index];
		}
		else
		{
			index = max(segment + 1, 0);
			vertices.insertAt(index, CurveVertex());
			vertex_count++;
			@p = vertices[index];
			p.init_control_points();
		}
		
		p.x = x;
		p.y = y;
		
		if(_type == CurveType::BSpline)
		{
			invalidated_b_spline_knots = true;
			invalidated_b_spline_vertices = true;
		}
		
		invalidate(index);
		
		return index;
	}
		
	/** Inserts a vertex in the given segment index and t value, attempting to preserve the curve shape. */
	int insert_vertex(const int segment, const float t)
	{
		if(segment < 0 || segment >= vertex_count - (_closed ? 0 : 1))
			return -1;
		
		switch(_type)
		{
			case CurveType::CatmullRom:
				return insert_vertex_catmull_rom(segment, t);
			case CurveType::QuadraticBezier:
				return insert_vertex_quadratic_bezier(segment, t);
			case CurveType::CubicBezier:
				return insert_vertex_cubic_bezier(segment, t);
			case CurveType::BSpline:
				return insert_vertex_b_spline(segment, t);
			case CurveType::Linear:
			default:
				return insert_vertex_linear(segment, t);
		}
		
		return -1;
	}
	
	private int insert_vertex_linear(const int segment, const float t)
	{
		float x, y;
		eval_linear_point(segment, t, x, y);
		return insert_vertex(segment, x, y);
	}
	
	private int insert_vertex_catmull_rom(const int segment, const float t)
	{
		float x, y;
		eval_catmull_rom_point(segment, t, x, y);
		return insert_vertex(segment, x, y);
	}
	
	private int insert_vertex_quadratic_bezier(const int segment, const float t)
	{
		CurveVertex@ p1 = vertices[segment];
		CurveVertex@ p3 = vert(segment + 1);
		CurveControlPoint@ p2 = p1.quad_control_point;
		
		// Linear fallback.
		if(p2.type == Square)
		{
			const int index = insert_vertex_linear(segment, t);
			CurveVertex@ b1 = vertices[index];
			b1.type = Square;
			b1.quad_control_point.type = Square;
			init_quadratic_bezier_control_points(true, index, 1);
			return index;
		}
		
		float a_p2x, a_p2y, m_x, m_y, b_p2x, b_p2y;
		float a_r2, m_r, b_r2;
		
		// Non-rational.
		if(p1.weight == p2.weight && p2.weight == p3.weight)
		{
			a_r2 = m_r = b_r2 = p1.weight;
			
			QuadraticBezier::split(
				p1.x, p1.y, p1.x + p2.x, p1.y + p2.y, p3.x, p3.y,
				t,
				a_p2x, a_p2y, m_x, m_y, b_p2x, b_p2y);
		}
		// Rational.
		else
		{
			QuadraticBezier::split(
				p1.x, p1.y, p1.x + p2.x, p1.y + p2.y, p3.x, p3.y,
				p1.weight, p2.weight, p3.weight,
				t,
				a_p2x, a_p2y, m_x, m_y, b_p2x, b_p2y,
				a_r2, m_r, b_r2);
		}
		
		const int index = insert_vertex(segment, m_x, m_y);
		CurveVertex@ b1 = vertices[index];
		
		p1.quad_control_point.type = Smooth;
		p1.quad_control_point.x = a_p2x - p1.x;
		p1.quad_control_point.y = a_p2y - p1.y;
		p1.quad_control_point.weight = a_r2;
		
		b1.weight = m_r;
		b1.quad_control_point.type = Smooth;
		b1.quad_control_point.x = b_p2x - m_x;
		b1.quad_control_point.y = b_p2y - m_y;
		b1.quad_control_point.weight = b_r2;
		
		return index;
	}
	
	private int insert_vertex_cubic_bezier(const int segment, const float t)
	{
		CurveVertex@ p1 = vertices[segment];
		CurveVertex@ p4 = vert(segment + 1);
		CurveControlPoint@ p2 = p1.cubic_control_point_2;
		CurveControlPoint@ p3 = p4.cubic_control_point_1;
		
		// Linear fallback.
		if(p2.type == Square && p3.type == Square)
		{
			const int index = insert_vertex_linear(segment, t);
			CurveVertex@ b1 = vertices[index];
			b1.type = Square;
			init_cubic_bezier_control_points(true, index, 1);
			b1.cubic_control_point_1.type = Square;
			b1.cubic_control_point_2.type = Square;
			return index;
		}
		
		// Quadratic fallback.
		if(p2.type == Square || p3.type == Square)
		{
			CurveControlPoint@ qp2 = p2.type == Square ? p3 : p2;
			CurveVertex@ p0 = p2.type == Square ? p4 : p1;
			
			float a_p2x, a_p2y, m_x, m_y, b_p2x, b_p2y;
			float a_r2, m_r, b_r2;
			
			// Non-rational.
			if(p1.weight == qp2.weight && qp2.weight == p4.weight)
			{
				a_r2 = m_r = b_r2 = p1.weight;
				
				QuadraticBezier::split(
					p1.x, p1.y, p0.x + qp2.x, p0.y + qp2.y, p4.x, p4.y,
					t,
					a_p2x, a_p2y, m_x, m_y, b_p2x, b_p2y);
			}
			// Rational.
			else
			{
				QuadraticBezier::split(
					p1.x, p1.y, p0.x + qp2.x, p0.y + qp2.y, p4.x, p4.y,
					p1.weight, qp2.weight, p4.weight,
					t,
					a_p2x, a_p2y, m_x, m_y, b_p2x, b_p2y,
					a_r2, m_r, b_r2);
			}
			
			const int index = insert_vertex(segment, m_x, m_y);
			CurveVertex@ b1 = vertices[index];
			b1.weight = m_r;
			
			CurveControlPoint@ a2 = p2.type == Square ? @b1.cubic_control_point_1 : @p2;
			a2.type = Smooth;
			a2.x = a_p2x - (p2.type == Square ? @b1 : @p1).x;
			a2.y = a_p2y - (p2.type == Square ? @b1 : @p1).y;
			a2.weight = a_r2;
			
			CurveControlPoint@ b2 = p2.type == Square ? @p3 : @b1.cubic_control_point_2;
			b2.type = Smooth;
			b2.x = b_p2x - (p2.type == Square ? @p4 : @b1).x;
			b2.y = b_p2y - (p2.type == Square ? @p4 : @b1).y;
			b2.weight = b_r2;
			
			if(p2.type == Square)
			{
				@b2 = @b1.cubic_control_point_2;
				b2.type = Square;
				b2.x = (p4.x - m_x) * 0.5;
				b2.y = (p4.y - m_y) * 0.5;
			}
			
			if(p3.type == Square)
			{
				@b2 = @b1.cubic_control_point_1;
				b2.type = Square;
				b2.x = (p1.x - m_x) * 0.5;
				b2.y = (p1.y - m_y) * 0.5;
			}
			
			return index;
		}
		
		float a_p2x, a_p2y, a_p3x, a_p3y, m_x, m_y;
		float b_p2x, b_p2y, b_p3x, b_p3y;
		float a_r2, a_r3, m_r, b_r2, b_r3;
		
		CubicBezier::split(
			p1.x, p1.y, p1.x + p2.x, p1.y + p2.y, p4.x + p3.x, p4.y + p3.y, p4.x, p4.y,
			p1.weight, p2.weight, p3.weight, p4.weight,
			t,
			a_p2x, a_p2y, a_p3x, a_p3y, m_x, m_y, b_p2x, b_p2y, b_p3x, b_p3y,
			a_r2, a_r3, m_r, b_r2, b_r3);
		
		const int index = insert_vertex(segment, m_x, m_y);
		CurveVertex@ b1 = vertices[index];
		b1.type = p1.type;
		
		p2.x = a_p2x - p1.x;
		p2.y = a_p2y - p1.y;
		p2.weight = a_r2;
		
		CurveControlPoint@ cp = @b1.cubic_control_point_1;
		cp.type = Smooth;
		cp.x = a_p3x - m_x;
		cp.y = a_p3y - m_y;
		cp.weight = a_r3;
		
		@cp = @b1.cubic_control_point_2;
		cp.type = Smooth;
		cp.x = b_p2x - m_x;
		cp.y = b_p2y - m_y;
		cp.weight = b_r2;
		
		p3.x = b_p3x - p4.x;
		p3.y = b_p3y - p4.y;
		p3.weight = b_r3;
		
		return index;
	}
	
	private int insert_vertex_b_spline(const int segment, const float t)
	{
		const int new_index = b_spline.insert_vertex_linear(b_spline_degree, b_spline_clamped, closed, segment, t);
		vertex_count++;
		
		invalidated_b_spline_knots = true;
		invalidated_b_spline_vertices = true;
		invalidate(new_index);
		
		return new_index;
	}
	
	//}
	
	/** Sets the type for the given vertex or control point.
	  * If the given point is a vertex, the control points on either side may be set depending on the curve type.
	  * Certain types are only applicable to vertices. */
	void set_control_type(CurveControlPoint@ point, const CurveControlType type, const bool set_mirror=true)
	{
		if(type == CurveControlType::None)
			return;
		
		CurveVertex@ v = cast<CurveVertex@>(point);
		
		if(@v != null)
		{
			const int index = vertices.findByRef(v);
			
			if(index != -1 && (_type == QuadraticBezier || _type == CubicBezier))
			{
				CurveControlPoint@ cp_l = _type == CubicBezier ? @v.cubic_control_point_1 : _closed || index > 0 ? @vert(index - 1).quad_control_point : null;
				CurveControlPoint@ cp_r = _type == CubicBezier ? @v.cubic_control_point_2 : @v.quad_control_point;
				
				if(@cp_l != null)
				{
					cp_l.type = type;
				}
				if(@cp_r != null)
				{
					cp_r.type = type;
				}
			}
			
			v.type = type;
			invalidate(index);
		}
		else
		{
			int index = vertices.findByRef(point.vertex);
			
			
			if(_type == CubicBezier)
			{
				if(@point == @point.vertex.cubic_control_point_1)
				{
					index--;
				}
				
				if(type != Smooth && set_mirror)
				{
					CurveControlPoint@ p2 = @point == @point.vertex.cubic_control_point_1
						? @point.vertex.cubic_control_point_2 : @point.vertex.cubic_control_point_1;
					
					if(p2.type == Smooth)
					{
						p2.type = Manual;
					}
				}
			}
			else if(_type == QuadraticBezier && type != Smooth && set_mirror)
			{
				CurveControlPoint@ p2 = @vert(index - 1).quad_control_point;
				if(p2.type == Smooth)
				{
					p2.type = Manual;
				}
				
				@p2 = @vert(index + 1).quad_control_point;
				if(p2.type == Smooth)
				{
					p2.type = Manual;
				}
			}
			
			point.type = type;
			invalidate((index % vertex_count + vertex_count) % vertex_count, true);
		}
	}
	
	/** Set the type/shape for control points of the given segment. Only applicable if the curve type is quadratic or cubic. */
	void set_segment_control_type(const int segment, const CurveControlType type)
	{
		if(_type != QuadraticBezier && _type != CubicBezier)
			return;
		
		CurveVertex@ v = vert(segment);
		CurveControlPoint@ p1 = _type == QuadraticBezier ? @v.quad_control_point : @v.cubic_control_point_2;
		CurveControlPoint@ p2 = _type == CubicBezier ? @vert(segment + 1).cubic_control_point_1 : null;
		
		set_control_type(p1, type);
		
		if(@p2 != null)
		{
			set_control_type(p2, type);
		}
	}
	
	/** Make sure to call `stop_drag_vertex` when done.
	  * @param x The x position the drag was initiated from (usually the mouse).
	  * @param y The y position the drag was initiated from (usually the mouse). */
	bool start_drag_vertex(CurveVertex@ vertex, const float x, const float y)
	{
		if(drag_curve.busy || drag_control_points_count != 0)
			return false;
		
		if(!drag_control_points[0].start_drag_vertex(this, vertex, x, y))
			return false;
		
		drag_control_points_count = 1;
		return true;
	}
	
	bool do_drag_vertex(const float x, const float y)
	{
		if(drag_control_points_count == 0)
			return false;
		
		if(!drag_control_points[0].do_drag_vertex(this, x, y))
			return false;
		
		return true;
	}
	
	bool stop_drag_vertex(const bool accept=true)
	{
		if(drag_control_points_count == 0)
			return false;
		
		drag_control_points_count = 0;
		
		if(!drag_control_points[0].stop_drag_vertex(this, accept))
			return false;
		
		return true;
	}
	
	/** Only applicable to quadratic or cubic curves.
	  * Does nothing if another drag is in progress - make sure to call `stop_drag_control_point` when done.
	  * @param x The x position the drag was initiated from (usually the mouse).
	  * @param y The y position the drag was initiated from (usually the mouse). */
	bool start_drag_control_point(CurveControlPoint@ point, const float x, const float y)
	{
		if(drag_curve.busy || drag_control_points_count != 0)
			return false;
		
		if(!drag_control_points[0].start_drag(this, point, x, y))
			return false;
		
		drag_control_points_count = 1;
		
		if(_type == QuadraticBezier)
		{
			drag_control_points[1].start_drag(this, point, x, y, 1);
			drag_control_points_count++;
		}
		
		return true;
	}
	
	bool do_drag_control_point(const float x, const float y, const ControlPointMirrorType mirror=Angle, const bool constrain_to_axis=false)
	{
		if(drag_control_points_count == 0)
			return false;
		
		if(!drag_control_points[0].do_drag(this, x, y, mirror, constrain_to_axis))
			return false;
		
		if(!constrain_to_axis && drag_control_points_count == 2)
		{
			drag_control_points[1].do_drag(this, x, y, mirror, drag_control_points_count == 1 && constrain_to_axis, false);
		}
		
		return true;
	}
	
	bool stop_drag_control_point(const bool accept=true)
	{
		if(drag_control_points_count == 0)
			return false;
		
		if(drag_control_points_count == 2)
		{
			drag_control_points[1].stop_drag(this, accept);
		}
		
		drag_control_points_count = 0;
		
		if(!drag_control_points[0].stop_drag(this, accept))
			return false;
		
		return true;
	}
	
	/** Only applicable to quadratic or cubic curves.
	  * Make sure to call `stop_drag_curve` when done.
	  * @param x The x position the drag was initiated from (usually the mouse).
	  * @param y The y position the drag was initiated from (usually the mouse). */
	bool start_drag_curve(const int segment, const float t, const float x, const float y, const CurveDragType drag_type=Advanced)
	{
		if(drag_control_points_count != 0)
			return false;
		
		CurveVertex @p1 = vert(segment);
		CurveVertex @p2 = vert(segment + 1);
		
		CurveType real_type = _type;
		switch(real_type)
		{
			case QuadraticBezier:
			{
				CurveControlPoint@ cp1 = p1.quad_control_point;
				if(cp1.type == Square)
				{
					real_type = Linear;
				}
			} break;
			case CubicBezier:
			{
				CurveControlPoint@ cp1 = p1.cubic_control_point_2;
				CurveControlPoint@ cp2 = p2.cubic_control_point_1;
				if(cp1.type == Square && cp2.type == Square)
				{
					real_type = Linear;
				}
				else if(cp1.type == Square || cp2.type == Square)
				{
					real_type = QuadraticBezier;
				}
			} break;
			case BSpline:
				return drag_curve.start_b_spline(this, segment, t, x, y);
			case Linear:
			case CatmullRom:
			default:
				drag_curve.busy = true;
				drag_curve.is_linear = true;
				drag_control_points[0].start_drag_vertex(this, p1, x, y);
				drag_control_points[1].start_drag_vertex(this, p2, x, y);
				return true;
		}
		
		if(!drag_curve.start(this, segment, t, x, y, drag_type))
			return false;
		
		if(real_type == QuadraticBezier)
		{
			drag_control_points[0].start_drag(this, drag_curve.cp1, x, y);
			drag_control_points[1].start_drag(this, drag_curve.cp1, x, y, 1);
		}
		else
		{
			drag_control_points[0].start_drag(this, drag_curve.cp1, x, y);
			drag_control_points[1].start_drag(this, drag_curve.cp2, x, y);
		}
		
		return true;
	}
	
	bool do_drag_curve(const float x, const float y, const bool update_mirrored_control_points=true)
	{
		if(!drag_curve.busy)
			return false;
		
		if(drag_curve.is_linear)
		{
			drag_control_points[0].do_drag_vertex(this, x, y);
			drag_control_points[1].do_drag_vertex(this, x, y);
			return true;
		}
		
		if(drag_curve.type == BSpline)
		{
			return drag_curve.update_b_spline(x, y);
		}
		
		if(!drag_curve.update(x, y))
			return false;
		
		if(update_mirrored_control_points)
		{
			drag_control_points[0].do_drag(this, x, y, ControlPointMirrorType::Angle, false, false);
			drag_control_points[1].do_drag(this, x, y, ControlPointMirrorType::Angle, false, false);
		}
		else
		{
			invalidate(drag_curve.segment, true);
		}
		
		return true;
	}
	
	bool stop_drag_curve(const bool accept=true)
	{
		if(!drag_curve.busy)
			return false;
		
		if(drag_curve.is_linear)
		{
			drag_curve.busy = false;
			drag_curve.is_linear = false;
			drag_control_points[0].stop_drag_vertex(this, accept);
			drag_control_points[1].stop_drag_vertex(this, accept);
			return true;
		}
		
		if(drag_curve.type == BSpline)
		{
			return drag_curve.stop_b_spline(accept);
		}
		
		if(!drag_curve.stop())
			return false;
		
		drag_control_points[0].stop_drag(this, accept);
		drag_control_points[1].stop_drag(this, accept);
		
		return true;
	}
	
	// --
	
	/** Returns the vertices/control points for the segment at `i` based whether the curve is open/closed, etc. */
	void get_segment_catmull_rom(const int i, CurveControlPoint@ &out p1, CurveVertex@ &out p2, CurveVertex@ &out p3, CurveControlPoint@ &out p4)
	{
		@p2 = @vertices[i];
		@p3 = vert(i + 1);
		
		@p1 = p2.type != Square
			? closed || i > 0
				? vert(i - 1)
				: _end_controls != Manual
					? this.p0.extrapolate(p2, p3,
						_end_controls == CurveEndControl::AutomaticAngle && vertex_count >= 3 ? @vertices[2] : null)
					: this.p0.added(p2, check_control_point_start())
			: p2;
		@p4 = p3.type != Square
			? closed || i < vertex_count - 2
				? vert(i + 2)
				: _end_controls != Manual
					? this.p3.extrapolate(p3, p2,
						_end_controls == CurveEndControl::AutomaticAngle && vertex_count >= 3 ? @vertices[vertex_count - 3] : null)
					: this.p3.added(p3, check_control_point_end())
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
	
	/** Returns the vertex at `i` wrapping around when < 0 or > vertex_count. */
	CurveVertex@ vert(const int i)
	{
		return vertex_count > 0
			? @vertices[(i % vertex_count + vertex_count) % vertex_count]
			: null;
	}
	
	/** Returns the vertex at `i + offset` wrapping around when < 0 or > vertex_count. */
	CurveVertex@ vert(CurveVertex@ vertex, const int offset=0)
	{
		if(@vertex == null || vertex_count == 0)
			return null;
		
		const int index = vertices.findByRef(vertex);
		if(index == -1)
			return null;
		
		return @vertices[((index + offset) % vertex_count + vertex_count) % vertex_count];
	}
	
	/** Returns an index based on the given segment and t value that better aligns with the actual curve.
	  * Only relevant for b-splines where the curve points may not line up exactly with vertices. */
	int get_adjusted_segment_index(const int segment, const float t)
	{
		if(_type != BSpline)
			return segment;
		
		return b_spline.get_adjusted_segment_index(
			_b_spline_degree, _b_spline_clamped, _closed,
			segment, t);
	}
	
	/** Returns the relative range of segments which may be affect when modifying a single vertex based on the curve type and settings. */
	void get_affected_vertex_offsets(int &out o1, int &out o2)
	{
		o1 = 0;
		o2 = 0;
		
		switch(_type)
		{
			case CurveType::BSpline:
				b_spline.get_affected_vertex_offsets(vertex_count, _b_spline_degree, _closed, o1, o2);
				break;
			case CurveType::CatmullRom:
				o1 = -2;
				o2 = 1;
				break;
			case CurveType::QuadraticBezier:
			case CurveType::CubicBezier:
			case CurveType::Linear:
			default:
				o1 = -1;
				break;
		}
	}
	
	// -- Bounding box methods --
	
	private void calc_bounding_box_linear()
	{
		const int end = segment_index_max;
		for(int i = 0; i <= end; i++)
		{
			CurveVertex@ p1 = vertices[i];
			
			if(p1.invalidated)
			{
				CurveVertex@ p2 = vert(i + 1);
				
				p1.x1 = p1.x < p2.x ? p1.x : p2.x;
				p1.y1 = p1.y < p2.y ? p1.y : p2.y;
				p1.x2 = p1.x > p2.x ? p1.x : p2.x;
				p1.y2 = p1.y > p2.y ? p1.y : p2.y;
			}
			
			if(p1.x1 < x1) x1 = p1.x1;
			if(p1.y1 < y1) y1 = p1.y1;
			if(p1.x2 > x2) x2 = p1.x2;
			if(p1.y2 > y2) y2 = p1.y2;
		}
	}
	
	private void calc_bounding_box_catmull_rom()
	{
		const int end = segment_index_max;
		for(int i = 0; i <= end; i++)
		{
			CurveVertex@ p2, p3;
			CurveControlPoint@ p1, p4;
			get_segment_catmull_rom(i, p1, p2, p3, p4);
			
			if(p2.invalidated)
			{
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
			}
			
			if(p2.x1 < x1) x1 = p2.x1;
			if(p2.y1 < y1) y1 = p2.y1;
			if(p2.x2 > x2) x2 = p2.x2;
			if(p2.y2 > y2) y2 = p2.y2;
		}
	}
	
	private void calc_bounding_box_quadratic_bezier()
	{
		const int end = segment_index_max;
		for(int i = 0; i <= end; i++)
		{
			CurveVertex@ p1 = @vertices[i];
			
			if(p1.invalidated)
			{
				const CurveVertex@ p3 = vert(i + 1);
				const CurveControlPoint@ p2 = p1.quad_control_point;
				
				// Linear fallback.
				if(p2.type == Square)
				{
					p1.x1 = p1.x < p3.x ? p1.x : p3.x;
					p1.y1 = p1.y < p3.y ? p1.y : p3.y;
					p1.x2 = p1.x > p3.x ? p1.x : p3.x;
					p1.y2 = p1.y > p3.y ? p1.y : p3.y;
				}
				else
				{
					
					if(p1.weight == p2.weight && p2.weight == p3.weight)
					{
						QuadraticBezier::bounding_box(
							p1.x, p1.y, p1.x + p2.x, p1.y + p2.y, p3.x, p3.y,
							p1.x1, p1.y1, p1.x2, p1.y2);
					}
					else
					{
						QuadraticBezier::bounding_box(
							p1.x, p1.y, p1.x + p2.x, p1.y + p2.y, p3.x, p3.y,
							p1.weight, p2.weight, p3.weight,
							p1.x1, p1.y1, p1.x2, p1.y2);
					}
				}
			}
			
			if(p1.x1 < x1) x1 = p1.x1;
			if(p1.y1 < y1) y1 = p1.y1;
			if(p1.x2 > x2) x2 = p1.x2;
			if(p1.y2 > y2) y2 = p1.y2;
		}
	}
	
	private void calc_bounding_box_cubic_bezier(const int samples=12, const float padding=0.5)
	{
		const int end = _closed ? vertex_count : vertex_count - 1;
		for(int i = 0; i < end; i++)
		{
			CurveVertex@ p1 = @vertices[i];
			
			if(p1.invalidated)
			{
				const CurveVertex@ p4 = vert(i + 1);
				const CurveControlPoint@ p2 = p1.cubic_control_point_2;
				const CurveControlPoint@ p3 = p4.cubic_control_point_1;
				
				// Linear fallback.
				if(p2.type == Square && p3.type == Square)
				{
					p1.x1 = p1.x < p4.x ? p1.x : p4.x;
					p1.y1 = p1.y < p4.y ? p1.y : p4.y;
					p1.x2 = p1.x > p4.x ? p1.x : p4.x;
					p1.y2 = p1.y > p4.y ? p1.y : p4.y;
				}
				// Quadratic fallback.
				else if(p2.type == Square || p3.type == Square)
				{
					const CurveControlPoint@ qp2 = p2.type == Square ? p4.cubic_control_point_1 : p1.cubic_control_point_2;
					const CurveControlPoint@ p0 = p2.type == Square ? p4 : p1;
					
					if(p1.weight == qp2.weight && qp2.weight == p4.weight)
					{
						QuadraticBezier::bounding_box(
							p1.x, p1.y, p0.x + qp2.x, p0.y + qp2.y, p4.x, p4.y,
							p1.x1, p1.y1, p1.x2, p1.y2);
					}
					else
					{
						QuadraticBezier::bounding_box(
							p1.x, p1.y, p0.x + qp2.x, p0.y + qp2.y, p4.x, p4.y,
							p1.weight, qp2.weight, p4.weight,
							p1.x1, p1.y1, p1.x2, p1.y2);
					}
				}
				else
				{
					if(p1.weight == p2.weight && p2.weight == p3.weight && p3.weight == p4.weight)
					{
						CubicBezier::bounding_box(
							p1.x, p1.y, p1.x + p2.x, p1.y + p2.y,
							p4.x + p3.x, p4.y + p3.y, p4.x, p4.y,
							p1.x1, p1.y1, p1.x2, p1.y2);
					}
					else
					{
						CubicBezier::bounding_box(
							p1.x, p1.y, p1.x + p2.x, p1.y + p2.y,
							p4.x + p3.x, p4.y + p3.y, p4.x, p4.y,
							p1.weight, p2.weight, p3.weight, p4.weight,
							p1.x1, p1.y1, p1.x2, p1.y2,
							samples, padding);
					}
				}
			}
			
			if(p1.x1 < x1) x1 = p1.x1;
			if(p1.y1 < y1) y1 = p1.y1;
			if(p1.x2 > x2) x2 = p1.x2;
			if(p1.y2 > y2) y2 = p1.y2;
		}
	}
	
	private void calc_bounding_box_b_spline()
	{
		b_spline.bounding_box_basic(
			vertex_count, _b_spline_degree, _closed,
			x1, y1, x2, y2);
	}
	
	// -- Util --
	
	private CurveVertex@ check_control_point_start()
	{
		if(control_point_start.type != None)
			return control_point_start;
		
		control_point_start.type = Square;
		return get_auto_control_start(control_point_start, CurveEndControl::AutomaticAngle)
			.relative_to(vert(0));
	}
	
	private CurveVertex@ check_control_point_end()
	{
		if(control_point_end.type != None)
			return control_point_end;
		
		control_point_end.type = Square;
		return get_auto_control_end(control_point_end, CurveEndControl::AutomaticAngle)
			.relative_to(vert(vertex_count - 1));
	}
	
	private void calc_segment_t(const int segment, const float t, float & out ts, int &out i)
	{
		const int max_i = _closed ? vertex_count - 1 : vertex_count - 2;
		
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
	
	private float calc_b_spline_t(const int segment, const float t)
	{
		return segment >= 0
			? (segment + (t > 0 && t < 1 ? t : t <= 0 ? 0 : 1)) / (_closed ? vertex_count : vertex_count - 1)
			: t;
	}
	
}
