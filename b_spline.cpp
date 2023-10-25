/** Make sure to call `set_vertices` and `generate_knots` before using, and after anything about the curve changes.
  * Ported from: https://github.com/pradeep-pyro/tinynurbs/tree/master */
class BSpline
{
	
	private array<float> knots(32);
	private int knots_length;
	
	private int vertex_count;
	private array<CurveVertex>@ vertices;
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
	
	/** Sets the vertices for this spline.
	  * Only needs to be called initially or once after the number of, position, or weight of any vertices change. */
	void set_vertices(
		array<CurveVertex>@ vertices, const int vertex_count,
		const int degree, const bool clamped, const bool closed)
	{
		this.vertex_count = vertex_count;
		@this.vertices = vertices;
		
		int v_count, degree_c;
		init_params(vertex_count, degree, clamped, closed, v_count, degree_c);
		
		// Compute points using homogenous coordinates.
		while(int(vertices_weighted.length) < v_count)
		{
			vertices_weighted.resize(vertices_weighted.length * 2);
		}
		
		// Offset the index just so that the start of the curve (t=0) aligns better with the start vertex.
		const int offset = closed ? degree_c / 2 : 0;
		
		for(int i = -offset; i < v_count - offset; i++)
		{
			CurvePointW@ vp = @vertices_weighted[i + offset];
			CurveVertex@ p = @vertices[(i % vertex_count + vertex_count) % vertex_count];
			vp.x = p.x * p.weight;
			vp.y = p.y * p.weight;
			vp.w = p.weight;
		}
	}
	
	/** Generates the correct set of uniform knots based on the given properties.
	  * See `eval` for a description of the properties.
	  * Must be called when the number of verices, the degree, clamped, or closed property have changed and after `set_vertices`. */
	void generate_knots(const int degree, const bool clamped, const bool closed)
	{
		int v_count, degree_c;
		init_params(vertex_count, degree, clamped, closed, v_count, degree_c);
		
		knots_length = v_count + degree_c + 1;
		while(int(knots.length) < knots_length)
		{
			knots.resize(knots.length * 2);
		}
		
		for(int i = 0; i < knots_length; i++)
		{
			knots[i] = !closed && clamped
				? min(max(i - degree_c, 0), knots_length - degree_c * 2 - 1)
				: i - degree_c;
		}
	}
	
	// -- Eval --
	
	/** Returns the point and normal at the given `t` value.
	  * Make sure `set_vertices` and `generate_knots` have been called at least once, or after anything about the curve changes.
	  * @param degree How smooth the curve is.
	  * @param clamped Whether or not the curve will touch the start and end vertices. Only applicable when open.
	  * @param closed If true the start and end vertices will be smoothly connected. */
	void eval(
		const int degree, const bool clamped, const bool closed,
		const float t, float &out x, float &out y, float &out w, float &out normal_x, float &out normal_y)
	{
		int v_count, degree_c;
		init_params(vertex_count, degree, clamped, closed, v_count, degree_c);
		
		switch(v_count)
		{
			case 2:
			{
				CurveVertex@ p1 = @vertices[0];
				CurveVertex@ p2 = @vertices[1];
				const float dx = p2.x - p1.x;
				const float dy = p2.y - p1.y;
				const float length = sqrt(dx * dx + dy * dy);
				x = p1.x + dx * t;
				y = p1.y + dy * t;
				normal_x = length != 0 ? dy / length : 0;
				normal_y = length != 0 ? -dx / length : 0;
				return;
			}
			case 1:
			{
				x = vertices[0].x;
				y = vertices[0].y;
				normal_x = 1;
				normal_y = 0;
				return;
			}
			case 0:
			{
				normal_x = 1;
				normal_y = 0;
				return;
			}
		}
		
		const float u = init_t(v_count, degree_c, closed, t);
		
		// Find span and corresponding non-zero basis functions.
		const int span = find_span(degree_c, u);
		calc_basis(degree_c, span, u);
		
		// Initialize result to 0s
		x = 0;
		y = 0;
		w = 0;
		
		for(int i = 0; i <= degree_c; i++)
		{
			const int j = span - degree_c + i;
			if(j < 0 || j >= v_count)
				continue;
			
			CurvePointW@ p = vertices_weighted[j];
			const float ni = basis_list[i];
			x += p.x * ni;
			y += p.y * ni;
			w += p.w * ni;
		}
		
		// Convert back to cartesian coordinates.
		if(w != 0)
		{
			x /= w;
			y /= w;
		}
		
		// Calculate the normal vector.
		curve_derivatives_rational(degree_c, closed, u, 1, span);

		CurvePointW@ du = @curve_ders[1];
		normal_x = du.y;
		normal_y = -du.x;
		
		const float length = sqrt(normal_x * normal_x + normal_y * normal_y);
		if(length != 0)
		{
			normal_x /= length;
			normal_y /= length;
		}
	}
	
	/** Returns the point at the given `t` value.
	  * Make sure `set_vertices` and `generate_knots` have been called at least once, or after anything about the curve changes. */
	void eval_point(
		const int degree, const bool clamped, const bool closed,
		const float t, float &out x, float &out y, float &out w)
	{
		int v_count, degree_c;
		init_params(vertex_count, degree, clamped, closed, v_count, degree_c);
		
		switch(v_count)
		{
			case 2:
			{
				CurveVertex@ p1 = @vertices[0];
				CurveVertex@ p2 = @vertices[1];
				const float dx = p2.x - p1.x;
				const float dy = p2.y - p1.y;
				x = p1.x + dx * t;
				y = p1.y + dy * t;
				return;
			}
			case 1:
			{
				x = vertices[0].x;
				y = vertices[0].y;
				return;
			}
			case 0:
			{
				x = 0;
				y = 0;
				return;
			}
		}
		
		const float u = init_t(v_count, degree_c, closed, t);
		
		// Find span and corresponding non-zero basis functions
		const int span = find_span(degree_c, u);
		calc_basis(degree_c, span, u);
		
		// Initialize result to 0s
		x = 0;
		y = 0;
		w = 0;
		
		if(v_count <= degree_c)
		{
			return;
		}
		
		// Compute point.
		for(int i = 0; i <= degree_c; i++)
		{
			CurvePointW@ p = vertices_weighted[span - degree_c + i];
			const float ni = basis_list[i];
			x += p.x * ni;
			y += p.y * ni;
			w += p.w * ni;
		}
		
		// Convert back to cartesian coordinates.
		x /= w;
		y /= w;
	}
	
	/** Returns the normal at the given `t` value. */
	void eval_normal(
		const int degree, const bool clamped, const bool closed,
		const float t, float &out normal_x, float &out normal_y)
	{
		int v_count, degree_c;
		init_params(vertex_count, degree, clamped, closed, v_count, degree_c);
		
		switch(v_count)
		{
			case 2:
			{
				CurveVertex@ p1 = @vertices[0];
				CurveVertex@ p2 = @vertices[1];
				const float dx = p2.x - p1.x;
				const float dy = p2.y - p1.y;
				const float length = sqrt(dx * dx + dy * dy);
				normal_x = length != 0 ? dy / length : 0;
				normal_y = length != 0 ? -dx / length : 0;
				return;
			}
			case 1:
			case 0:
			{
				normal_x = 1;
				normal_y = 0;
				return;
			}
		}
		
		const float u = init_t(v_count, degree_c, closed, t);
		
		// Calculate the normal vector.
		curve_derivatives_rational(degree_c, closed, u, 1);

		CurvePointW@ du = @curve_ders[1];
		normal_x = du.y;
		normal_y = -du.x;
		
		const float length = sqrt(normal_x * normal_x + normal_y * normal_y);
		if(length != 0)
		{
			normal_x /= length;
			normal_y /= length;
		}
	}
	
	// -- Bounding boxes --
	
	/** Calculates an approximate bounding box by simply finding the min and max of all vertices. */
	void bounding_box_simple(
		const int vertex_count, const int degree, const bool closed,
		float &out x1, float &out y1, float &out x2, float &out y2)
	{
		if(vertex_count == 0)
			return;
		
		const int degree_c = clamp(degree, 2, vertex_count - 1);
		const int o1 = closed ? -degree_c / 2 : -(degree_c + 1) / 2;
		const int o2 = closed ? (degree_c + 1) / 2 : (degree_c + 1) / 2 + 1;
		
		x1 = y1 = INFINITY;
		x2 = y2 = -INFINITY;
		
		for(int i = 0; i < vertex_count; i++)
		{
			CurveVertex@ v = vertices[i];
			if(v.x < x1) x1 = v.x;
			if(v.y < y1) y1 = v.y;
			if(v.x > x2) x2 = v.x;
			if(v.y > y2) y2 = v.y;
			
			v.x1 = v.x2 = v.x;
			v.y1 = v.y2 = v.y;
			
			for(int j = i + o1; j <= i + o2; j++)
			{
				if(j == i)
					continue;
				if(!closed && (j < 0 || j >= vertex_count))
					continue;
				
				CurveVertex@ v2 = vertices[(j % vertex_count + vertex_count) % vertex_count];
				if(v2.x < v.x1) v.x1 = v2.x;
				if(v2.y < v.y1) v.y1 = v2.y;
				if(v2.x > v.x2) v.x2 = v2.x;
				if(v2.y > v.y2) v.y2 = v2.y;
			}
		}
	}
	
	/** Calculates an approximate bounding box by taking the min/max of the vertices and first and last arc segment position.
	  * Gives a tighter bounding box than `bounding_box_simple`, but requires arc lengths to have been updated. */
	void bounding_box_basic(
		const int vertex_count, const int degree, const bool closed,
		float &out x1, float &out y1, float &out x2, float &out y2)
	{
		if(vertex_count == 0)
			return;
		
		const int degree_c = clamp(degree, 2, vertex_count - 1);
		const int o1 = (closed ? -degree_c / 2 : -(degree_c + 1) / 2) + 1;
		const int o2 = (closed ? (degree_c + 1) / 2 : (degree_c + 1) / 2 + 1) - 1;
		
		x1 = y1 = INFINITY;
		x2 = y2 = -INFINITY;
		
		const int end = closed ? vertex_count : vertex_count - 1;
		for(int i = 0; i < end; i++)
		{
			CurveVertex@ v = vertices[i];
			
			v.x1 = v.x2 = v.x;
			v.y1 = v.y2 = v.y;
			
			for(int j = 0; j < 2; j++)
			{
				CurveArc@ arc = j == 0 ? v.arcs[0] : v.arcs[v.arc_count - 1];
				if(arc.x < v.x1) v.x1 = arc.x;
				if(arc.y < v.y1) v.y1 = arc.y;
				if(arc.x > v.x2) v.x2 = arc.x;
				if(arc.y > v.y2) v.y2 = arc.y;
			}
			
			for(int j = i + o1; j <= i + o2; j++)
			{
				if(j == i)
					continue;
				if(!closed && (j < 0 || j >= vertex_count))
					continue;
				
				CurveVertex@ v2 = vertices[(j % vertex_count + vertex_count) % vertex_count];
				if(v2.x < v.x1) v.x1 = v2.x;
				if(v2.y < v.y1) v.y1 = v2.y;
				if(v2.x > v.x2) v.x2 = v2.x;
				if(v2.y > v.y2) v.y2 = v2.y;
			}
			
			if(v.x1 < x1) x1 = v.x1;
			if(v.y1 < y1) y1 = v.y1;
			if(v.x2 > x2) x2 = v.x2;
			if(v.y2 > y2) y2 = v.y2;
		}
	}
	
	/** Returns the range indicating how many vertices on each side of any given vertex will affect that vertex. */
	void get_affected_vertex_offsets(const int vertex_count, const int degree, const bool closed, int &out o1, int &out o2)
	{
		const int degree_c = clamp(degree, 2, vertex_count - 1);
		o1 = closed ? -(degree_c + 1) / 2 : -(degree_c + 1) / 2 - 1;
		o2 = closed ? degree_c / 2 : (degree_c + 1) / 2;
	}
	
	// -- Modification --
	
	/** Inserts a vertex at the given t value. The curve shape may not be fully preserved after insertion.
	  * `set_vertices` and `generate_knots` need to be called before and after. */
	int insert_vertex(
		const int degree, const bool clamped, const bool closed,
		const float t)
	{
		int v_count, degree_c;
		init_params(vertex_count, degree, clamped, closed, v_count, degree_c);
		const float u = init_t(v_count, degree_c, closed, t);
		
		const int span = find_span(degree_c, u);
		const int multiplicity = knot_multiplicity(u);
		if(multiplicity >= degree_c)
			return vertex_count;
		
		const int offset = closed ? degree_c / 2 : 0;
		
		// Copy affected control points
		array<CurvePointW>@ tmp = @curve_wders;
		while(int(tmp.length) < degree_c - multiplicity + 1)
		{
			tmp.resize(tmp.length * 2);
		}
		for(int i = 0; i < degree_c - multiplicity + 1; i++)
		{
			tmp[i] = vertices_weighted[span - degree_c + i + offset];
		}
		
		// Shift vertices up.
		vertex_count++;
		while(int(vertices_weighted.length) < vertex_count)
		{
			vertices_weighted.resize(vertices_weighted.length * 2);
		}
		
		const int w_index = span - multiplicity;
		for(int i = vertex_count - 1; i >= w_index; i--)
		{
			vertices_weighted[i + offset] = vertices_weighted[i - 1 + offset];
		}
		
		// Modify affected control point
		const int li = span - degree_c + 1;
		for(int i = 0; i < degree_c - multiplicity; i++)
		{
			const float a = (u - knots[li + i]) / (knots[i + span + 1] - knots[li + i]);
			CurvePointW@ c = @tmp[i];
			CurvePointW@ c1 = @tmp[i + 1];
			c.x = (1 - a) * c.x + a * c1.x;
			c.y = (1 - a) * c.y + a * c1.y;
			c.w = (1 - a) * c.w + a * c1.w;
		}
		
		knots_length++;
		while(int(knots.length) < knots_length)
		{
			knots.resize(knots.length * 2);
		}
		
		// Shift knots up and insert new.
		knots.insertAt(span + 1, u);
		
		// Insert vertex and convert back to cartesian coordinates
		if(w_index < vertex_count)
		{
			vertices.insertAt(w_index, CurveVertex(0, 0));
			vertices[w_index].init_control_points();
		}
		else
		{
			vertices.resize(vertices.length + 1);
		}
		
		for(int i = li; i <= w_index; ++i)
		{
			CurvePointW@ vp = @vertices_weighted[i + offset];
			CurveVertex@ p = @vertices[i % vertex_count];
			vp = tmp[i - li];
			p.x = vp.x / vp.w;
			p.y = vp.y / vp.w;
			p.weight = vp.w;
		}
		
		return vertex_count;
	}
	
	/** Inserts a vertex at the given segment and t value by simply linearly interpolating the vertices.
	  * `set_vertices` and `generate_knots` will need to be called after.
	  * @return The newly inserted vertex index. */
	int insert_vertex_linear(
		const int degree, const bool clamped, const bool closed,
		int segment, float t)
	{
		int v_count, degree_c;
		init_params(vertex_count, degree, clamped, closed, v_count, degree_c);
		const float u = init_t(v_count, degree_c, closed, t);
		
		CurveVertex@ p1 = vertices[segment];
		CurveVertex@ p2 = vertices[(segment + 1) % vertex_count];
		
		if(segment + 1 < vertex_count)
		{
			vertices.insertAt(segment + 1, CurveVertex(0, 0));
		}
		else
		{
			vertices.resize(vertices.length + 1);
		}
		vertex_count++;
		
		CurveVertex@ p = vertices[segment + 1];
		p.init_control_points();
		
		if(closed || segment < vertex_count - 2)
		{
			p.x = p1.x + (p2.x - p1.x) * (0.5 + t * 0.5);
			p.y = p1.y + (p2.y - p1.y) * (0.5 + t * 0.5);
			p.weight = p1.weight + (p2.weight - p1.weight) * t;
		}
		else
		{
			p.x = p1.x;
			p.y = p1.y;
			p.weight = p1.weight;
		}
		
		if(closed || segment + 3 < vertex_count)
		{
			@p = vertices[(segment + 2) % vertex_count];
			@p2 = vertices[(segment + 3) % vertex_count];
			p.x = p.x + (p2.x - p.x) * t * 0.5;
			p.y = p.y + (p2.y - p.y) * t * 0.5;
			p.weight = p.weight + (p2.weight - p.weight) * t;
		}
		
		const int tt =  int(segment + t + 1);
		return tt % vertex_count;
	}
	
	/** Returns an index based on the given segment and t value that better aligns with the actual curve.
	  * since curve points may not line up exactly with vertices. */
	int get_adjusted_segment_index(
		const int degree, const bool clamped, const bool closed,
		int segment, float t)
	{
		int v_count, degree_c;
		init_params(vertex_count, degree, clamped, closed, v_count, degree_c);
		
		if(degree_c % 2 != 0)
			return segment;
		
		get_adjusted_segment_index(closed, segment, t, segment, t);
		
		return (int(segment + t - 0.5) % vertex_count + vertex_count) % vertex_count;
	}
	
	private void get_adjusted_segment_index(
		const bool closed, const int segment, const float t,
		int &out out_segment, float &out out_t)
	{
		out_segment = segment;
		out_t = t - 0.5;
		
		if(!closed && out_segment == 0 && t < 0.5)
		{
			out_t = (out_t + 0.5) * 0.5;
			if(out_segment > 0)
			{
				out_segment--;
			}
		}
		else if(out_t < 0)
		{
			if(closed || out_segment > 0)
			{
				out_t += 1;
				out_segment = ((out_segment - 1) % vertex_count + vertex_count) % vertex_count;
			}
			else
			{
				out_t = 0;
			}
		}
	}
	
	// --
	
	private void init_params(
		const int vertex_count,
		const int degree, const bool clamped, const bool closed,
		int &out out_v_count, int &out out_degree)
	{
		out_degree = clamp(degree, 2, vertex_count - 1);
		out_v_count = closed ? vertex_count + out_degree + 1 : vertex_count;
	}
	
	private float init_t(const int v_count, const int degree, const bool closed, const float t)
	{
		return t * (closed ? 1 - 1.0 / (vertex_count + 1) : 1.0) * (v_count - degree);
	}
	
	private void curve_derivatives_rational(
		const int degree, const bool closed,
		const float u, const int num_ders, const int span=-1)
	{
		// Derivatives of Cw.
		curve_derivatives(vertices_weighted, degree, num_ders, u, span);
		
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
	
	private void curve_derivatives(
		array<CurvePointW>@ vertices_weighted, const int degree,
		const int num_ders, const float u, int span=-1)
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
		if(span == -1)
		{
			span = find_span(degree, u);
		}
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
				const float den = right[r + 1] + left[j - r];
				const float temp = den != 0 ? basis_list[r] / den : 0;
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
	
	private int knot_multiplicity(const float u)
	{
		int multiplicity = 0;
		
		for(int i = knots_length - 1; i >= 0; i--)
		{
			if(closeTo(u, knots[i]))
			{
				multiplicity++;
			}
		}
		
		return multiplicity;
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
		while(arr.length < n1)
		{
			arr.resize(arr.length >= 32 ? arr.length * 2 : 32);
		}
		
		for(uint i = 0; i < n1; i++)
		{
			array<float>@ arr2 = @arr[i];
			while(arr2.length < n2)
			{
				arr2.resize(arr2.length >= 32 ? arr2.length * 2 : 32);
			}
		}
	}
	
}
