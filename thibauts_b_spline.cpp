
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
	
