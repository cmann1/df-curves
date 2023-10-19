namespace CubicBezier
{
	
	// -- Eval --
	
	/** Calculate the position and normal at the given t value for a non-rational quadratic bezier curve defined by
	  * two vertices (`p1` and `p4`) and two control point (`p2` and `p3`). */
	void eval(
		const float p1x, const float p1y, const float p2x, const float p2y,
		const float p3x, const float p3y, const float p4x, const float p4y,
		const float t, float &out x, float &out y, float &out normal_x, float &out normal_y,
		const bool normalise=true)
	{
		// Calculate the point.
		const float u = 1 - t;
		const float tt = t*t;
		const float tt3 = tt*t;
		const float uu = u*u;
		const float uuu = uu*u;
		
		// Calculate point.
		x = uuu * p1x + 3*uu*t*p2x + 3*u*tt*p3x + tt3*p4x;
		y = uuu * p1y + 3*uu*t*p2y + 3*u*tt*p3y + tt3*p4y;
		
		// Calculate normal.
		normal_x =
			-3*p1y*uu +
			3*p2y*(3*uu - 2*u) +
			3*p3y*(2*t - 3*tt) +
			3*p4y*tt;
		normal_y = -(
			-3*p1x*uu +
			3*p2x*(3*uu - 2*u) +
			3*p3x*(2*t - 3*tt) +
			3*p4x*tt);
		
		if(normalise)
		{
			const float length = sqrt(normal_x * normal_x + normal_y * normal_y);
			if(length != 0)
			{
				normal_x /= length;
				normal_y /= length;
			}
		}
	}
	
	/** Calculate the position at the given t value for a non-rational quadratic bezier curve defined by
	  * two vertices (`p1` and `p4`) and two control point (`p2` and `p3`). */
	void eval_point(
		const float p1x, const float p1y, const float p2x, const float p2y,
		const float p3x, const float p3y, const float p4x, const float p4y,
		const float t, float &out x, float &out y)
	{
		// Calculate the point.
		const float u = 1 - t;
		const float tt = t*t;
		const float tt3 = tt*t;
		const float uu = u*u;
		const float uuu = uu*u;
		
		// Calculate point.
		x = uuu * p1x + 3*uu*t*p2x + 3*u*tt*p3x + tt3*p4x;
		y = uuu * p1y + 3*uu*t*p2y + 3*u*tt*p3y + tt3*p4y;
	}
	
	/** Calculate the normal at the given t value for a non-rational quadratic bezier curve defined by
	  * two vertices (`p1` and `p4`) and two control point (`p2` and `p3`). */
	void eval_normal(
		const float p1x, const float p1y, const float p2x, const float p2y,
		const float p3x, const float p3y, const float p4x, const float p4y,
		const float t, float &out normal_x, float &out normal_y, const bool normalise=true)
	{
		const float u = 1 - t;
		const float tt = t*t;
		const float uu = u*u;
		
		// Calculate normal.
		normal_x =
			-3*p1y*uu +
			3*p2y*(3*uu - 2*u) +
			3*p3y*(2*t - 3*tt) +
			3*p4y*tt;
		normal_y = -(
			-3*p1x*uu +
			3*p2x*(3*uu - 2*u) +
			3*p3x*(2*t - 3*tt) +
			3*p4x*tt);
		
		if(normalise)
		{
			const float length = sqrt(normal_x * normal_x + normal_y * normal_y);
			if(length != 0)
			{
				normal_x /= length;
				normal_y /= length;
			}
		}
	}
	
	// -- Bounding boxes --
	
	/** Calculate the bounding box of a non-rational quadratic bezier curve defined by
	  * two vertices (`p1` and `p4`) and two control point (`p2` and `p3`). */
	void bounding_box(
		const float p1x, const float p1y, const float p2x, const float p2y,
		const float p3x, const float p3y, const float p4x, const float p4y,
		float &out x1, float &out y1, float &out x2, float &out y2)
	{
		x1 = p1x < p4x ? p1x : p4x;
		y1 = p1y < p4y ? p1y : p4y;
		x2 = p4x > p1x ? p4x : p1x;
		y2 = p4y > p1y ? p4y : p1y;
		
		const float ax = 3*(-p1x + 3*p2x - 3*p3x + p4x);
		const float ay = 3*(-p1y + 3*p2y - 3*p3y + p4y);
		const float bx = 6*(p1x - 2*p2x + p3x);
		const float by = 6*(p1y - 2*p2y + p3y);
		const float cx = 3*(p2x - p1x);
		const float cy = 3*(p2y - p1y);
		
		// Calculate the x and y roots by plugging the a, b, and c coefficients into the quadratic formula.
		const float dscx = sqrt(bx*bx - 4*ax*cx);
		const float dscy = sqrt(by*by - 4*ay*cy);
		const float t1x = abs(ax) > 0.01 ? (-bx + dscx)/(2*ax) : abs(bx) > 0.01 ? - cx/bx : -1;
		const float t2x = abs(ax) > 0.01 ? (-bx - dscx)/(2*ax) : abs(bx) > 0.01 ? - cx/bx : -1;
		const float t1y = abs(ay) > 0.01 ? (-by + dscy)/(2*ay) : abs(by) > 0.01 ? - cy/by : -1;
		const float t2y = abs(ay) > 0.01 ? (-by - dscy)/(2*ay) : abs(by) > 0.01 ? - cy/by : -1;
		
		if(t1x >= 0 && t1x <= 1)
		{
			const float u = 1 - t1x;
			const float tt = t1x*t1x;
			const float tt3 = tt*t1x;
			const float uu = u*u;
			const float uuu = uu*u;
			
			const float x = uuu*p1x + 3*uu*t1x*p2x + 3*u*tt*p3x + tt3*p4x;
			
			if(x < x1) x1 = x;
			if(x > x2) x2 = x;
		}
		
		if(t2x >= 0 && t2x <= 1)
		{
			const float u = 1 - t2x;
			const float tt = t2x*t2x;
			const float tt3 = tt*t2x;
			const float uu = u*u;
			const float uuu = uu*u;
			const float x = uuu*p1x + 3*uu*t2x*p2x + 3*u*tt*p3x + tt3*p4x;
			
			if(x < x1) x1 = x;
			if(x > x2) x2 = x;
		}
		
		if(t1y >= 0 && t1y <= 1)
		{
			const float u = 1 - t1y;
			const float tt = t1y*t1y;
			const float tt3 = tt*t1y;
			const float uu = u*u;
			const float uuu = uu*u;
			const float y = uuu*p1y + 3*uu*t1y*p2y + 3*u*tt*p3y + tt3*p4y;
			
			if(y < y1) y1 = y;
			if(y > y2) y2 = y;
		}
		
		if(t2y >= 0 && t2y <= 1)
		{
			const float u = 1 - t2y;
			const float tt = t2y*t2y;
			const float tt3 = tt*t2y;
			const float uu = u*u;
			const float uuu = uu*u;
			const float y = uuu*p1y + 3*uu*t2y*p2y + 3*u*tt*p3y + tt3*p4y;
			
			if(y < y1) y1 = y;
			if(y > y2) y2 = y;
		}
	}
	
}
