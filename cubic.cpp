namespace CubicBezier
{
	
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
	
}
