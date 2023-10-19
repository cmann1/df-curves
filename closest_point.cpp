namespace Curve
{
	
	/**
	  * @param max_distance If > 0, only points closer than this will be returned. Can also potentially reduce the amount of work needed
	  *   by skipping segments that are out of range with simple bounds checks.
	  * @param threshold When the distance between tested points becomes smaller than this, stop looking.
	  * @param arc_length_interpolation If true can provide more accurate reults near loops or where the arc subdivisions do not have enough resolution
	  *   at the cost of more curve evaluations.
	  *   First it finds the closest point on the linear arc segments, samples the curve at the interpolated t value, and projects that point back onto the
	  *   normal vector giving a much better guess at how close a segment is to the desired point.
	  * @param adjust_initial_binary_factor If true can potentially reduce the number of iterations needed to reach the threshold by skewing
	  *   the binary search range on the initial guess.
	  * @param interpolate_result If true interpolates the t value of the end result which can result in smoother results with larger threshold values.
	  * @param x1 y1 x2 y2 The bounding box of the curve. Only required when `max_distance` > 0.
	  * @return true if a point was found within `max_distance` */
	bool closest_point(
		array<CurveVertex> vertices, const int vertex_count, const bool closed,
		EvalPointFunc@ eval_point,
		const float x, const float y, int &out segment_index, float &out out_t, float &out out_x, float &out out_y,
		const float max_distance=0, float threshold=1,
		const bool arc_length_interpolation=true,
		const bool adjust_initial_binary_factor=true,
		const bool interpolate_result=true,
		const float x1=-INFINITY, const float y1=-INFINITY, const float x2=INFINITY, const float y2=INFINITY)
	{
				if(vertex_count == 0 || vertices[0].arc_count == 0)
			return false;
		
		const int end = closed ? vertex_count : vertex_count - 1;
		
		if(max_distance > 0 && (
			x < x1 - max_distance || x > x2 + max_distance ||
			y < y1 - max_distance || y > y2 + max_distance))
			return false;
		
		// -- Step 1. Find the closest arc segment.
		
		segment_index = -1;
		int closest_arc_index = -1;
		CurveArc@ clostest_arc = null;
		float closest_arc_length = 0;
		float dist = INFINITY;
		float dist_interpolated = INFINITY;
		bool is_interpolated = false;
		float guess_dist = -1;
		
		for(int i = 0; i < end; i++)
		{
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
				float c_dist_interpolated = INFINITY;
				float c_guess_dist = -1;
				float c_length = c.length;
				float c_x = c.x;
				float c_y = c.y;
				float c_t = c.t;
				
				// Project the point onto the current arc segment to find a more accurate initial guess.
				if(arc_length_interpolation && j > 0 && (c.dx != 0 || c.dy != 0))
				{
					CurveArc@ c0 = v.arcs[j - 1];
					float arc_local_t = ((x - c0.x) * c.dx + (y - c0.y) * c.dy) / c.length_sqr;
					
					if(arc_local_t > 0 && arc_local_t < 1)
					{
						const float linear_x = c0.x + c.dx * arc_local_t;
						const float linear_y = c0.y + c.dy * arc_local_t;
						float arc_t = c0.t + (c.t - c0.t) * arc_local_t;
						
						float arc_x, arc_y;
						eval_point(i, arc_t, arc_x, arc_y);
						
						// Take the interpolated curve point (which could be farther away) and project it back onto the
						// perpendicular line from the closest linear point to get something that's hopefully closer to the curve and desired point.
						float curve_guess_x, curve_guess_y;
						project(arc_x - linear_x, arc_y - linear_y, x - linear_x, y - linear_y, curve_guess_x, curve_guess_y);
						curve_guess_x += linear_x;
						curve_guess_y += linear_y;
						
						c_dist_interpolated = min(
							(curve_guess_x - x) * (curve_guess_x - x) + (curve_guess_y - y) * (curve_guess_y - y),
							(arc_x - x) * (arc_x - x) + (arc_y - y) * (arc_y - y)
						);
						
						c_guess_dist = (arc_x - curve_guess_x) * (arc_x - curve_guess_x) + (arc_y - curve_guess_y) * (arc_y - curve_guess_y);
						
						c_x = arc_x;
						c_y = arc_y;
						c_t = arc_t;
					}
				}
				
				const float c_dist = (x - c_x) * (x - c_x) + (y - c_y) * (y - c_y);
				
				if((c_dist_interpolated < c_dist ? c_dist_interpolated : c_dist) > dist_interpolated)
					continue;
				
				is_interpolated = c_dist_interpolated != INFINITY;
				
				out_t = c_t;
				out_x = c_x;
				out_y = c_y;
				segment_index = i;
				closest_arc_index = j;
				closest_arc_length = c_length;
				dist = c_dist;
				dist_interpolated = is_interpolated ? c_dist_interpolated : c_dist;
				guess_dist = c_guess_dist;
			}
		}
		
		if(segment_index == -1)
			return false;
		
		// -- Step 2. Using the closest arc segment and the two surrounding points, do a binary search to find progressively closer
		//            points until the threshold is reached.
		
		CurveVertex@ v = vertices[segment_index];
		out_t += segment_index;
		
		// Initialise bounds for binary search.
		
		const int si1 = closest_arc_index > 0 || is_interpolated ? segment_index
			: segment_index > 0 ? segment_index - 1
			: segment_index;
		const int si2 = closest_arc_index < v.arc_count - 1 || is_interpolated ? segment_index
			: closed || segment_index < end - 1 ? segment_index + 1
			: segment_index;
		
		float t1, t2;
		float p1x, p1y, p2x, p2y;
		
		if(closest_arc_index > 0)
		{
			CurveArc@ c1 = v.arcs[closest_arc_index - 1];
			t1 = si1 + c1.t;
			p1x = c1.x;
			p1y = c1.y;
		}
		else if(segment_index > 0)
		{
			CurveArc@ c1 = vertices[segment_index - 1].arc_from_end(1);
			t1 = si1 + c1.t;
			p1x = c1.x;
			p1y = c1.y;
		}
		else
		{
			t1 = out_t;
			p1x = out_x;
			p1y = out_y;
		}
		
		if(is_interpolated)
		{
			CurveArc@ c2 = v.arcs[closest_arc_index];
			t2 = si2 + c2.t;
			p2x = c2.x;
			p2y = c2.y;
		}
		else if(closest_arc_index < v.arc_count - 1)
		{
			CurveArc@ c2 = v.arcs[closest_arc_index + 1];
			t2 = si2 + c2.t;
			p2x = c2.x;
			p2y = c2.y;
		}
		else if(closed || segment_index < end - 1)
		{
			CurveArc@ c2 = vertices[(segment_index + 1) % vertex_count].arc_from_start(1);
			t2 = si2 + c2.t;
			p2x = c2.x;
			p2y = c2.y;
		}
		else
		{
			t2 = out_t;
			p2x = out_x;
			p2y = out_y;
		}
		
		threshold *= threshold;
		
		// Interpolating the initial guess usually makes it more acurate.
		// Making the bounds tighter initially and slowly increasing back to 0.5 seems to save on iterations and reach the threshold somewhat faster.
		float binary_search_factor;
		
		if(adjust_initial_binary_factor)
		{
			binary_search_factor = arc_length_interpolation && closest_arc_length != 0
				? map_clamped(sqrt(guess_dist) / closest_arc_length, 0.1, 0.5, 0.15, 0.95)
				: 0.15;
		}
		else
		{
			binary_search_factor = 0.5;
		}
		
		do
		{
			float p1mx, p1my;
			float p2mx, p2my;
			
			// Left side.
			const float t1m = out_t + (t1 - out_t) * binary_search_factor;
			const int i1 = (int(t1m) % vertex_count + vertex_count) % vertex_count;
			eval_point(i1, fraction(t1m), p1mx, p1my);
			const float dist1m = (p1mx - x) * (p1mx - x) + (p1my - y) * (p1my - y);
			
			// Right side.
			const float t2m = out_t + (t2 - out_t) * binary_search_factor;
			const int i2 = (int(t2m) % vertex_count + vertex_count) % vertex_count;
			eval_point(i2, fraction(t2m), p2mx, p2my);
			const float dist2m = (p2mx - x) * (p2mx - x) + (p2my - y) * (p2my - y);
			
			// Mid point is closest.
			if(dist <= dist1m && dist <= dist2m)
			{
				t1 = t1m;
				p1x = p1mx;
				p1y = p1my;
				t2 = t2m;
				p2x = p2mx;
				p2y = p2my;
			}
			// Left point is closest.
			else if(dist1m < dist2m)
			{
				t2 = out_t;
				p2x = out_x;
				p2y = out_y;
				out_t = t1m;
				out_x = p1mx;
				out_y = p1my;
				dist = dist1m;
			}
			// Right point is closest.
			else
			{
				t1 = out_t;
				p1x = out_x;
				p1y = out_y;
				out_t = t2m;
				out_x = p2mx;
				out_y = p2my;
				dist = dist2m;
			}
			
			if(binary_search_factor > 0.5)
			{
				binary_search_factor = 0.25;
			}
			else if(binary_search_factor < 0.5)
			{
				binary_search_factor = binary_search_factor + (0.5 - binary_search_factor) * 0.25;
			}
		}
		while((p2x - p1x) * (p2x - p1x) + (p2y - p1y) * (p2y - p1y) > threshold && !closeTo(t1, t2));
		
		if(interpolate_result)
		{
			const float dx = p2x - p1x;
			const float dy = p2y - p1y;
			
			if(dx != 0 || dy != 0)
			{
				const float it = clamp01(((x - p1x) * dx + (y - p1y) * dy) / (dx * dx + dy * dy));
				out_t = t1 + (t2 - t1) * it;
				segment_index = (int(out_t) % vertex_count + vertex_count) % vertex_count;
				out_t = fraction(out_t);
				eval_point(segment_index, out_t, out_x, out_y);
			}
		}
		else
		{
			out_t = fraction(out_t);
		}
		
		if(max_distance > 0 && (x - out_x) * (x - out_x) + (y - out_y) * (y - out_y) > max_distance * max_distance)
			return false;
		
		return true;
	}
	
}
