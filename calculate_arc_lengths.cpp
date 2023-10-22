#include 'EvalFunc.cpp';

namespace Curve
{
	
	/** Subdivides a curve using the given eval func. Each `CurveVertex` is considered a separate segment of the curve,
	  * and the results are stored in the `length` and `arcs` property of each vertex.
	  * @param vertices The vertices defining a curve made up of multiple segments.
	  * @param vertex_count The number of vertices.
	  * @param closed Is the curve closed or open.
	  * @param eval The curve evaluation function.
	  * @param only_invalidated If true, only vertices with the `invalidate` field set to true will be recalculated.
	  * @param division_count How many sections each segment/vertex will be broken into. The highter this number the more accurate the results.
	  * @param angle_min If > 0, will subdivide each arc segment if the angle between the start and end of the segment is greater than this angle (radians).
	  * @param max_stretch_factor If > 0 and < 1 attempts to make divisions more uniform in extreme cases, e.g. by weighted control points.
	  *   The smaller the better.
	  *   This is done by comparing the real length to the interpolated arc segment length at the specific t values and forcing a subdivision if it's larger than this.
	  * @param length_min If > 0 will stop subdividing when the arc length becomes smaller than this value.
	  * @param max_subdivisions If > 0 limits how many subdivisions are allowed per segment.
	  * @param angle_max If > 0 any segment where the angle (radians) is greater than this a subdivision will be forced
	  *   regardless of `max_subdivisions`.
	  *   Can help improve accuracy around tight corners without increasing the resolution or adaptive parameters a lot.
	  * @param length_max If > 0 segments larger than this will subdivide regardless `max_subdivisions`. Not recommended as the number of subdivisions will be proportional
	  *   to the length of the curve, but will ensure a more even distribution regardless.
	  * @return The total length of the curve. */
	float calculate_arc_lengths(
		array<CurveVertex>@ vertices, const int vertex_count, const bool closed,
		EvalFunc@ eval, const bool only_invalidated, const int division_count,
		const float angle_min=0, const float max_stretch_factor=0,
		const float length_min=0, const int max_subdivisions=0,
		const float angle_max=0, const float length_max=0)
	{
		float total_length = 0;
		
		const int v_count = closed ? vertex_count - 1 : vertex_count - 2;
		for(int i = 0; i <= v_count; i++)
		{
			CurveVertex@ v = vertices[i];
			
			if(only_invalidated && !v.invalidated)
			{
				total_length += v.length;
				continue;
			}
			
			array<CurveArc>@ arcs = @v.arcs;
			uint arc_count = 0;
			v.length = total_length;
			
			while(division_count >= int(arcs.length))
			{
				arcs.resize(arcs.length < 8 ? 8 : arcs.length * 2);
			}
			
			float t1 = 0;
			float x1 = 0;
			float y1 = 0;
			float n1x = 0;
			float n1y = 0;
			
			float arc_length_sqr = 0, arc_length = 0;
			float t_length = 0;
			float dx = 0, dy = 0, nx = 0, ny = 0;
			
			for(int j = 0; j <= division_count; j++)
			{
				const float t2 = float(j) / division_count;
				
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
						angle_min, max_stretch_factor,
						length_min,
						angle_min > 0 || length_min > 0 || max_stretch_factor > 0 ? max_subdivisions : 0,
						angle_max, length_max,
						arc_length_sqr, arc_length, total_length, t_length,
						dx, dy, nx, ny);
				}
				
				if(arc_count + 1 >= arcs.length)
				{
					arcs.resize(arcs.length * 2);
				}
				
				CurveArc@ arc = @arcs[arc_count++];
				arc.t = t2;
				arc.x = x2;
				arc.y = y2;
				arc.length_sqr = arc_length_sqr;
				arc.length = arc_length;
				arc.total_length = total_length;
				arc.t_length = t_length;
				arc.dx = dx;
				arc.dy = dy;
				arc.nx = nx;
				arc.ny = ny;
				
				t1 = t2;
				x1 = x2;
				y1 = y2;
				n1x = n2x;
				n1y = n2y;
			}
			
			v.arc_count = int(arc_count);
			v.length = total_length - v.length;
		}
		
		return total_length;
	}
	
	/** Internal method - recursively subdivides and adds arc segments between t1 and t2. */
	uint _add_arc_length(
		EvalFunc@ eval, array<CurveArc>@ arcs, uint arc_count,
		const int segment_index, const float t1, const float t2,
		const float x1, const float y1, const float n1x, const float n1y,
		const float x2, const float y2, const float n2x, const float n2y,
		const float total_length,
		const float angle_min, const float max_stretch_factor,
		const float length_min, const int max_subdivisions,
		const float angle_max, const float length_max,
		float &out out_arc_length_sqr, float &out out_arc_length, float &out out_total_length, float &out out_t_length,
		float &out out_dx, float &out out_dy, float &out out_nx, float &out out_ny)
	{
		out_dx = x2 - x1;
		out_dy = y2 - y1;
		out_arc_length_sqr = out_dx * out_dx + out_dy * out_dy;
		out_arc_length = sqrt(out_arc_length_sqr);
		out_nx = out_arc_length != 0 ? out_dy / out_arc_length : 0.0;
		out_ny = out_arc_length != 0 ? -out_dx / out_arc_length : 0.0;
		out_total_length = total_length + out_arc_length;
		out_t_length = t2 - t1;
		
		const bool allow_subdivide = max_subdivisions > 0 && (length_min <= 0 || out_arc_length > length_min);
		const float angle_diff = angle_min > 0 && allow_subdivide
			? acos(clamp(n1x * n2x + n1y * n2y, -1.0, 1.0)) : 0.0;
		
		const bool subdivide = out_arc_length != 0 && (
			angle_max > 0 && acos(clamp(n1x * n2x + n1y * n2y, -1.0, 1.0)) > angle_max ||
			length_max > 0 && out_dx * out_dx + out_dy * out_dy > length_max * length_max ||
			allow_subdivide && angle_diff > angle_min);
		
		// Calculate the mid point between t1 and t2.
		const float tm = (t1 + t2) * 0.5;
		float mx, my, nmx, nmy;
		
		if(!subdivide)
		{
			if(out_arc_length == 0 || max_stretch_factor <= 0 || closeTo(tm, t2))
				return arc_count;
			
			eval(segment_index, tm, mx, my, nmx, nmy, true);
			const float real_length = sqrt((mx - x1) * (mx - x1) + (my - y1) * (my - y1));
			
			if(abs(real_length - out_arc_length * 0.5) / (out_arc_length * 0.5) < max_stretch_factor)
				return arc_count;
		}
		else
		{
			eval(segment_index, tm, mx, my, nmx, nmy, true);
		}
		
		// Subdivide the left.
		arc_count = _add_arc_length(
			eval, arcs, arc_count,
			segment_index, t1, tm,
			x1, y1, n1x, n1y,
			mx, my, nmx, nmy,
			total_length,
			angle_min, max_stretch_factor,
			length_min, max_subdivisions - 1,
			angle_max, length_max,
			out_arc_length_sqr, out_arc_length, out_total_length, out_t_length,
			out_dx, out_dy, out_nx, out_ny);
		
		// Add the mid point.
		
		if(arc_count + 1 >= arcs.length)
		{
			arcs.resize(arcs.length * 2);
		}
		
		CurveArc@ arc = @arcs[arc_count++];
		arc.t = tm;
		arc.x = mx;
		arc.y = my;
		arc.length_sqr = out_arc_length_sqr;
		arc.length = out_arc_length;
		arc.total_length = out_total_length;
		arc.t_length = out_t_length;
		arc.dx = out_dx;
		arc.dy = out_dy;
		arc.nx = out_nx;
		arc.ny = out_ny;
		
		// Subdivide the right.
		arc_count = _add_arc_length(
			eval, arcs, arc_count,
			segment_index, tm, t2,
			mx, my, nmx, nmy,
			x2, y2, n2x, n2y,
			out_total_length,
			angle_min, max_stretch_factor,
			length_min, max_subdivisions - 1,
			angle_max, length_max,
			out_arc_length_sqr, out_arc_length, out_total_length, out_t_length,
			out_dx, out_dy, out_nx, out_ny);
		
		return arc_count;
	}
	
}
