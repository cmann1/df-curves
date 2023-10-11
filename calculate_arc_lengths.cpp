namespace Curve
{
	
	/** A function to evaluate a curve at the given segment index and t value.
	  * @param segment The segment index.
	  * @param t The t value within the segment in the ragfe 0..1.
	  * @param x The x value of the returned point on the curve.
	  * @param y The y value of the returned point on the curve.
	  * @param normal_x The x value of the normal of the returned point on the curve.
	  * @param normal_y The y value of the normal of the returned point on the curve.
	  * @param normalise Whether or mot the returned normal values should be normalised to length 1. */
	funcdef void EvalFunc(const int, const float, float &out, float &out, float &out, float &out, const bool);
	
	/** Subdivides a curve using the given eval func. Each `CurveVertex` is considered a separate segment of the curve,
	  * and the results are stored in the `length` and `arcs` property of each vertex.
	  * @param vertices The vertices defining a curve made up of multiple segments.
	  * @param vertex_count The number of vertices.
	  * @param closed Is the curve closed or open.
	  * @param eval The curve evaluation function.
	  * @param divisions How many sections each segment/vertex will be broken into. The highter this number the more accurate the results.
	  * @param adaptive_angle If > 0, will subdivide each arc segment if the angle between the start and end of the segment is greater than this angle (radians).
	  * @param adaptive_max_subdivisions Must be > 0. Limits how many adaptive subdivisions are allowed.
	  * @param adaptive_min_length If > 0 will also stop subdividing when the arc length becomes smaller than this value.
	  * @param adaptive_angle_max If > 0 any segment where the angle (radians) is greater than this a subdivision will be forced
	  *   regardless of the adaptive angle, subdivision limit, or length.
	  *   Can help improve accuracy around tight corners without increasing the resolution or adaptive parameters a lot.
	  * @return The total length of the curve. */
	float calculate_arc_lengths(
		array<CurveVertex>@ vertices, const int vertex_count, const bool closed,
		EvalFunc@ eval, const int divisions,
		const float adaptive_angle=0, const int adaptive_max_subdivisions=0, const float adaptive_min_length=0,
		const float adaptive_angle_max=0)
	{
		float total_length = 0;
		
		const int v_count = closed ? vertex_count - 1 : vertex_count - 2;
		for(int i = 0; i <= v_count; i++)
		{
			CurveVertex@ v = vertices[i];
			array<CurveArc>@ arcs = @v.arcs;
			uint arc_count = 0;
			
			while(divisions >= int(arcs.length))
			{
				arcs.resize(arcs.length < 8 ? 8 : arcs.length * 2);
			}
			
			float t1 = 0;
			float x1 = 0;
			float y1 = 0;
			float n1x = 0;
			float n1y = 0;
			
			for(int j = 0; j <= divisions; j++)
			{
				const float t2 = float(j) / divisions;
				
				float x2, y2;
				float n2x, n2y;
				eval(i, t2, x2, y2, n2x, n2y, true);
				
				if(j > 0)
				{
					arc_count = _add_arc_length(
						eval, arcs, arc_count,
						i, t1, t2,
						x1, y1, n1x, n1y,
						x2, y2, n2x, n2y,
						total_length,
						adaptive_angle, adaptive_angle > 0 ? adaptive_max_subdivisions : 0, adaptive_min_length,
						adaptive_angle_max,
						total_length);
				}
				
				if(arc_count + 1 >= arcs.length)
				{
					arcs.resize(arcs.length * 2);
				}
				
				CurveArc@ arc = @arcs[arc_count++];
				arc.t = t2;
				arc.x = x2;
				arc.y = y2;
				arc.length = total_length;
				
				t1 = t2;
				x1 = x2;
				y1 = y2;
				n1x = n2x;
				n1y = n2y;
			}
			
			v.arc_count = int(arc_count);
		}
		
		return total_length;
	}
	
	/** Internal method ignore.
	  * Recursively subdivides and adds arc segments between t1 and t2. */
	uint _add_arc_length(
		EvalFunc@ eval, array<CurveArc>@ arcs, uint arc_count,
		const int segment_index, const float t1, const float t2,
		const float x1, const float y1, const float n1x, const float n1y,
		const float x2, const float y2, const float n2x, const float n2y,
		const float total_length,
		const float adaptive_angle, const int adaptive_max_subdivisions, const float adaptive_min_length,
		const float adaptive_angle_max,
		float &out out_length)
	{
		const float dx = x2 - x1;
		const float dy = y2 - y1;
		float arc_length = sqrt(dx * dx + dy * dy);
		out_length = total_length + arc_length;
		
		if(
			// `adaptive_angle_max` takes priority over other conditions.
			(
				adaptive_angle_max <= 0 ||
				// 
				acos(clamp(n1x * n2x + n1y * n2y, -1.0, 1.0)) <= adaptive_angle_max
				
			) && (
				// The subdivision limit has been reached.
				adaptive_max_subdivisions <= 0 ||
				// The minimum length for a subdivide segment has been reached.
				adaptive_min_length > 0 && arc_length <= adaptive_min_length ||
				// The angle is not low enough to warrant a subdivision.
				acos(clamp(n1x * n2x + n1y * n2y, -1.0, 1.0)) <= adaptive_angle
			))
			return arc_count;
		
		// Calculate the mid point between t1 and t2.
		const float tm = (t1 + t2) * 0.5;
		float mx, my, nmx, nmy;
		eval(segment_index, tm, mx, my, nmx, nmy, true);
		
		// Subdivide the left.
		arc_count = _add_arc_length(
			eval, arcs, arc_count,
			segment_index, t1, tm,
			x1, y1, n1x, n1y,
			mx, my, nmx, nmy,
			total_length,
			adaptive_angle, adaptive_max_subdivisions - 1, adaptive_min_length,
			adaptive_angle_max,
			arc_length);
		
		// Add the mid point.
		
		if(arc_count + 1 >= arcs.length)
		{
			arcs.resize(arcs.length * 2);
		}
		
		CurveArc@ arc = @arcs[arc_count++];
		arc.t = tm;
		arc.x = mx;
		arc.y = my;
		arc.length = arc_length;
		
		// Subdivide the right.
		arc_count = _add_arc_length(
			eval, arcs, arc_count,
			segment_index, tm, t2,
			mx, my, nmx, nmy,
			x2, y2, n2x, n2y,
			arc.length,
			adaptive_angle, adaptive_max_subdivisions - 1, adaptive_min_length,
			adaptive_angle_max,
			out_length);
		
		return arc_count;
	}
	
}
