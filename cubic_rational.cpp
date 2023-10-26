namespace CubicBezier
{
	
	/** Calculate the position and normal at the given t value for a rational quadratic bezier curve defined by
	  * two vertices (`p1` and `p4`) and two control point (`p2` and `p3`), and the corresponding ratios/weights. */
	void eval(
		const float p1x, const float p1y, const float p2x, const float p2y,
		const float p3x, const float p3y, const float p4x, const float p4y,
		const float r1, const float r2, const float r3, const float r4,
		const float t, float &out x, float &out y, float &out normal_x, float &out normal_y)
	{
		// Calculate the point.
		const float u = 1 - t;
		const float tt = t*t;
		const float tt3 = tt*t;
		const float uu = u*u;
		const float uuu = uu*u;
		
		const float f1 = uuu*r1;
		const float f2 = 3*uu*t*r2;
		const float f3 = 3*u*tt*r3;
		const float f4 = tt3 * r4;
		const float basis = f1 + f2 + f3 + f4;
			
		// Calculate point.
		x = (f1*p1x + f2*p2x + f3*p3x + f4*p4x) / basis;
		y = (f1*p1y + f2*p2y + f3*p3y + f4*p4y) / basis;
			
		// Calculate Normal.
		const float ut2 = 2*u*t;
		const float basis2 = 3*(
			+ ut2*(r3 - r2)
			+ uu*(r2 - r1)
			- tt*(r3 - r4)
		);
		normal_x =
			(
				3*(
					+ uu*(p2y*r2 - p1y*r1)
					+ ut2*(p3y*r3 - p2y*r2)
					- tt*(p3y*r3 - p4y*r4)
				)
			) / basis
			- (basis2*(p1y*f1 + p2y*f2 + p3y*f3 + p4y*f4)) / (basis*basis);
		normal_y = -(
			(
				3*(
					+ uu*(p2x*r2 - p1x*r1)
					+ ut2*(p3x*r3 - p2x*r2)
					- tt*(p3x*r3 - p4x*r4)
				)
			) / basis
			- (basis2*(p1x*f1 + p2x*f2 + p3x*f3 + p4x*f4)) / (basis*basis));
		
		const float length = sqrt(normal_x * normal_x + normal_y * normal_y);
		if(length != 0)
		{
			normal_x /= length;
			normal_y /= length;
		}
	}
	
	/** Calculate the position at the given t value for a rational quadratic bezier curve defined by
	  * two vertices (`p1` and `p4`) and two control point (`p2` and `p3`), and the corresponding ratios/weights. */
	void eval_point(
		const float p1x, const float p1y, const float p2x, const float p2y,
		const float p3x, const float p3y, const float p4x, const float p4y,
		const float r1, const float r2, const float r3, const float r4,
		const float t, float &out x, float &out y)
	{
		const float u = 1 - t;
		const float tt = t*t;
		const float tt3 = tt*t;
		const float uu = u*u;
		const float uuu = uu*u;
		
		const float f1 = uuu*r1;
		const float f2 = 3*uu*t*r2;
		const float f3 = 3*u*tt*r3;
		const float f4 = tt3 * r4;
		const float basis = f1 + f2 + f3 + f4;
			
		// Calculate point.
		x = (f1*p1x + f2*p2x + f3*p3x + f4*p4x) / basis;
		y = (f1*p1y + f2*p2y + f3*p3y + f4*p4y) / basis;
	}
	
	/** Calculate the normal at the given t value for a rational quadratic bezier curve defined by
	  * two vertices (`p1` and `p4`) and two control point (`p2` and `p3`), and the corresponding ratios/weights. */
	void eval_normal(
		const float p1x, const float p1y, const float p2x, const float p2y,
		const float p3x, const float p3y, const float p4x, const float p4y,
		const float r1, const float r2, const float r3, const float r4,
		const float t, float &out normal_x, float &out normal_y)
	{
		const float u = 1 - t;
		const float tt = t*t;
		const float tt3 = tt*t;
		const float uu = u*u;
		const float uuu = uu*u;
		
		const float f1 = uuu*r1;
		const float f2 = 3*uu*t*r2;
		const float f3 = 3*u*tt*r3;
		const float f4 = tt3 * r4;
		const float basis = f1 + f2 + f3 + f4;
			
		// Calculate Normal.
		const float ut2 = 2*u*t;
		const float basis2 = 3*(
			+ ut2*(r3 - r2)
			+ uu*(r2 - r1)
			- tt*(r3 - r4)
		);
		normal_x =
			(
				3*(
					+ uu*(p2y*r2 - p1y*r1)
					+ ut2*(p3y*r3 - p2y*r2)
					- tt*(p3y*r3 - p4y*r4)
				)
			) / basis
			- (basis2*(p1y*f1 + p2y*f2 + p3y*f3 + p4y*f4)) / (basis*basis);
		normal_y = -(
			(
				3*(
					+ uu*(p2x*r2 - p1x*r1)
					+ ut2*(p3x*r3 - p2x*r2)
					- tt*(p3x*r3 - p4x*r4)
				)
			) / basis
			- (basis2*(p1x*f1 + p2x*f2 + p3x*f3 + p4x*f4)) / (basis*basis));
		
		const float length = sqrt(normal_x * normal_x + normal_y * normal_y);
		if(length != 0)
		{
			normal_x /= length;
			normal_y /= length;
		}
	}
	
	/** Returns the ratio/weight at the given t value. */
	float eval_ratio(
		const float p1x, const float p1y, const float p2x, const float p2y,
		const float p3x, const float p3y, const float p4x, const float p4y,
		const float r1, const float r2, const float r3, const float r4,
		const float t)
	{
		const float u = 1 - t;
		const float tt = t*t;
		const float tt3 = tt*t;
		const float uu = u*u;
		const float uuu = uu*u;
		
		return uuu*r1 + 3*uu*t*r2 + 3*u*tt*r3 + tt3 * r4;
	}
	
}
