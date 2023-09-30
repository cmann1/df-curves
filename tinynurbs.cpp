
	/// The smoothness of the b-spline. Must be > 1 and < `vertex_count`.
	[persist] int b_spline_degree = 2;
	
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
		const int degree = clamp(b_spline_degree, 2, vertex_count - 1);
		const float u = t * (vertex_count - degree);
		
		// Generate knots.
		knots_length = vertex_count + degree + 1;
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
		//knots[0] = 0;
		//knots[1] = 0;
		//knots[2] = 0;
		//knots[3] = 1;
		//knots[4] = 2;
		//knots[5] = 3;
		//knots[6] = 3;
		//knots[7] = 3;
		
		// Compute points using homogenous coordinates.
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
		if(int(curve_ders.length) < num_ders + 1)
		{
			curve_ders.resize(num_ders + 1);
		}
		
		// Compute homogenous coordinates of control points.
		if(int(v.length) < vertex_count)
		{
			v.resize(vertex_count);
		}
		for(int i = 0; i < vertex_count; i++)
		{
			const CurveVertex@ p = @vertices[i];
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
		const int degree = clamp(b_spline_degree, 2, vertex_count - 1);
		const float u = t * (vertex_count - degree);
		
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
	
