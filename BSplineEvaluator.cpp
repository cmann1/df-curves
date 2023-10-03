/// https://github.com/pradeep-pyro/tinynurbs/tree/master
class BSplineEvaluator
{
	
	private array<float> knots(32);
	private int knots_length;
	
	private array<CurvePointW> vertices_weighted(32);
	private array<CurvePointW> curve_wders(32);
	private array<CurvePointW> curve_ders(32);
	private array<array<float>> ndu;
	private array<array<float>> ders;
	private array<array<float>> b_a;
	private array<float> basis_list(32);
	private array<CurvePoint> v_ders(32);
	private array<float> w_ders(32);
	private array<float> left(32);
	private array<float> right(32);
	
	private bool invalidated_weights = true;
	
	void eval(
		const float t, float &out x, float &out y, float &out normal_x, float &out normal_y,
		array<CurveVertex>@ vertices, const int vertex_count,
		const int degrees, const bool clamped, const bool closed,
		const EvalReturnType return_type=EvalReturnType::Both)
	{
		int v_count, clamp_val, degree;
		init_params(v_count, clamp_val, degree, vertex_count, degrees, clamped, closed);
		const float u = clamp_val + t * (v_count - degree - (closed ? clamp_val : 0));
		
		if(invalidated_weights)
		{
			// Compute points using homogenous coordinates.
			while(int(vertices_weighted.length) < v_count)
			{
				vertices_weighted.resize(vertices_weighted.length * 2);
			}
			
			for(int i = 0; i < v_count; i++)
			{
				CurvePointW@ vp = @vertices_weighted[i];
				CurveVertex@ p = @vertices[i % vertex_count];
				vp.x = p.x * p.weight;
				vp.y = p.y * p.weight;
				vp.w = p.weight;
			}
			
			invalidated_weights = false;
		}
		
		if(return_type == EvalReturnType::Both || return_type == EvalReturnType::Point)
		{
			// Find span and corresponding non-zero basis functions
			const int span = find_span(degree, u);
			calc_basis(degree, span, u);
			
			// Initialize result to 0s
			x = 0;
			y = 0;
			float w = 0;
			
			// Compute point.
			for(int i = 0; i <= degree; i++)
			{
				CurvePointW@ p = vertices_weighted[span - degree + i];
				const float ni = basis_list[i];
				x += p.x * ni;
				y += p.y * ni;
				w += p.w * ni;
			}
			
			// Convert back to cartesian coordinates.
			x /= w;
			y /= w;
		}
		else
		{
			x = 0;
			y = 0;
		}
		
		// Calculate the normal vector.
		if(return_type == EvalReturnType::Both || return_type == EvalReturnType::Normal)
		{
			curve_derivatives_rational(u, 1, degree, closed);

			CurvePointW@ du = @curve_ders[1];
			normal_x = du.y;
			normal_y = -du.x;
		}
	}
	
	void generate_knots(
		array<CurveVertex>@ vertices, const int vertex_count,
		const int degree, const bool clamped, const bool closed)
	{
		int v_count, clamp_val, degree_valid;
		init_params(v_count, clamp_val, degree_valid, vertex_count, degree, clamped, closed);
		
		const uint st = get_time_us();
		knots_length = v_count + degree_valid + 1;
		while(int(knots.length) < knots_length)
		{
			knots.resize(knots.length * 2);
		}
		
		for(int i = 0; i < knots_length; i++)
		{
			// A clamped b-spline touches the first and last vertices.
			// To do this make sure the first and last knot are repeated `degree + 1` times.
			knots[i] = min(max(i - degree_valid + clamp_val, 0), knots_length - (degree_valid - clamp_val) * 2 - 1);
		}
	}
	
	/// Call any time the number of, position, or weight of any vertices changes.
	void invalidate_vertices()
	{
		invalidated_weights = true;
	}
	
	private void init_params(
		int &out out_v_count, int &out out_clamp_val, int &out out_degree,
		const int vertex_count,
		const int degree, const bool clamped, const bool closed)
	{
		out_v_count = closed ? vertex_count + 3 : vertex_count;
		out_clamp_val = clamped && !closed ? 0 : 1;
		out_degree = clamp(degree, 2, vertex_count - 1);
	}
	
	private void curve_derivatives_rational(
		const float u, const int num_ders,
		const int degree, const bool closed)
	{
		// Derivatives of Cw.
		curve_derivatives(vertices_weighted, num_ders, degree, u);
		
		// Split into coordinates and weights.
		while(v_ders.length < vertices_weighted.length)
		{
			v_ders.resize(v_ders.length * 2);
			w_ders.resize(w_ders.length * 2);
		}
		for(uint i = 0; i < curve_wders.length; i++)
		{
			const CurvePointW@ vp = @curve_wders[i];
			CurvePoint@ vdp = v_ders[i];
			
			vdp.x = vp.x;
			vdp.y = vp.y;
			w_ders[i] = vp.w;
		}
		
		// Compute rational derivatives.
		
		while(int(curve_ders.length) < num_ders + 1)
		{
			curve_ders.resize(curve_ders.length * 2);
		}
		
		const float w0 = w_ders[0];
		for(int i = 0; i <= num_ders; i++)
		{
			CurvePoint@ v = v_ders[i];
			for(int j = 1; j <= i; j++)
			{
				CurvePointW@ cd = @curve_ders[i - j];
				const float w = w_ders[j];
				const float binomial = calc_binomial(i, j) * w;
				v.x -= binomial * cd.x;
				v.y -= binomial * cd.y;
			}
			
			CurvePointW@ cd = @curve_ders[i];
			cd.x = v.x / w0;
			cd.y = v.y / w0;
		}
	}
	
	private void curve_derivatives(array<CurvePointW>@ vertices_weighted, const int num_ders, const int degree, const float u)
	{
		if(int(curve_wders.length) < num_ders + 1)
		{
			curve_wders.resize(num_ders + 1);
		}
		
		// Assign higher order derivatives to zero.
		for(int i = degree + 1; i <= num_ders; i++)
		{
			CurvePointW@ cd = @curve_wders[i];
			cd.x = 0;
			cd.y = 0;
			cd.w = 0;
		}
		
		// Find the span and corresponding non-zero basis functions & derivatives.
		const int span = find_span(degree, u);
		calc_der_basis(degree, span, u, num_ders);
		
		// Compute first num_ders derivatives.
		const int du = num_ders < degree ? num_ders : degree;
		for(int i = 0; i <= du; i++)
		{
			CurvePointW@ cd = @curve_wders[i];
			cd.x = 0;
			cd.y = 0;
			cd.w = 0;
			
			for (int j = 0; j <= degree; j++)
			{
				CurvePointW@ p = @vertices_weighted[span - degree + j];
				const float der = ders[i][j];
				
				cd.x += p.x * der;
				cd.y += p.y * der;
				cd.w += p.w * der;
			}
		}
	}
	
	private void calc_basis(const int degree, const int span, const float u)
	{
		const uint size = degree + 1;
		
		while(left.length < size)
		{
			left.resize(left.length * 2);
			right.resize(right.length * 2);
		}
		
		while(basis_list.length < size)
		{
			basis_list.resize(basis_list.length * 2);
		}
		
		float saved = 0;
		
		for(uint i = 0; i < size; i++)
		{
			left[i] = 0;
			right[i] = 0;
			basis_list[i] = 0;
		}
		
		basis_list[0] = 1.0;
		
		for(int j = 1; j <= degree; j++)
		{
			left[j] = u - knots[span + 1 - j];
			right[j] = knots[span + j] - u;
			saved = 0.0;
			
			for(int r = 0; r < j; r++)
			{
				const float temp = basis_list[r] / (right[r + 1] + left[j - r]);
				basis_list[r] = saved + right[r + 1] * temp;
				saved = left[j - r] * temp;
			}
			basis_list[j] = saved;
		}
	}
	
	private void calc_der_basis(const int degree, const int span, const float u, const int num_ders)
	{
		const uint size = degree + 1;
		
		while(left.length < size)
		{
			left.resize(left.length * 2);
			right.resize(right.length * 2);
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
		
		ensure_array_2(@b_a, 2, degree + 1);
		
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
	
	private int find_span(const int degree, const float u)
	{
		// Index of last control point.
		const int n = knots_length - degree - 2;
		// For values of u that lies outside the domain.
		if (u >= knots[n + 1])
			return n;
		if (u <= knots[degree])
			return degree;
		
		// Binary search.
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
	
	private int calc_binomial(const int n, const int k)
	{
		if(k > n)
			return 0;
		
		int result = 1;
		
		for(int i = 1; i <= k; ++i)
		{
			result *= (n + 1 - i);
			result /= i;
		}
		
		return result;
	}
	
	private void ensure_array_2(array<array<float>>@ arr, const uint n1, const uint n2)
	{
		if(arr.length >= n1)
			return;
		
		const uint si = arr.length;
		
		while(arr.length < n1)
		{
			arr.resize(arr.length >= 32 ? arr.length * 2 : 32);
		}
		
		for(uint i = si; i < n1; i++)
		{
			array<float>@ arr2 = @arr[i];
			while(arr2.length < n2)
			{
				arr2.resize(arr2.length >= 32 ? arr2.length * 2 : 32);
			}
		}
	}
	
}
