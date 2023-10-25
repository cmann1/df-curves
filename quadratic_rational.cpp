namespace QuadraticBezier
{
	
	/** Calculate the position and normal at the given t value for a rational quadratic bezier curve defined by
	  * two vertices (`p1` and `p3`) and a control point (`p2`), and the corresponding ratios/weights. */
	void eval(
		const float p1x, const float p1y, const float p2x, const float p2y, const float p3x, const float p3y,
		const float r1, const float r2, const float r3,
		const float t, float &out x, float &out y, float &out w, float &out normal_x, float &out normal_y)
	{
		// Calculate the point.
		const float u = 1 - t;
		const float tt = t*t;
		const float uu = u*u;
		const float ut2 = 2*u*t;
		
		const float f1 = r1*uu;
		const float f2 = r2*ut2;
		const float f3 = r3*tt;
		w = f1 + f2 + f3;
		
		// Calculate point.
		x = (f1*p1x + f2*p2x + f3*p3x) / w;
		y = (f1*p1y + f2*p2y + f3*p3y) / w;
		
		// Calculate normal.
		const float d2 = 2*(
			+ u*(r2 - r1)
			- t*(r2 - r3));
		normal_x =
			(2*(u*(r2*p2y - r1*p1y) + t*(r3*p3y - r2*p2y))) / w -
			(d2*(r1*p1y*uu + 2*r2*p2y*t*u + r3*p3y*tt)) / (w*w);
		normal_y = -(
			(2*(u*(r2*p2x - r1*p1x) + t*(r3 * p3x - r2 * p2x))) / w -
			(d2 * (r1 * p1x * uu + 2 * r2 * p2x * t * u + r3 * p3x * tt)) / (w * w));
		
		const float length = sqrt(normal_x * normal_x + normal_y * normal_y);
		if(length != 0)
		{
			normal_x /= length;
			normal_y /= length;
		}
	}
	
	/** Calculate the position at the given t value for a rational quadratic bezier curve defined by
	  * two vertices (`p1` and `p3`) and a control point (`p2`), and the corresponding ratios/weights. */
	void eval_point(
		const float p1x, const float p1y, const float p2x, const float p2y, const float p3x, const float p3y,
		const float r1, const float r2, const float r3,
		const float t, float &out x, float &out y, float &out w)
	{
		// Calculate the point.
		const float u = 1 - t;
		const float tt = t*t;
		const float uu = u*u;
		const float ut2 = 2*u*t;
		
		const float f1 = r1 * uu;
		const float f2 = r2 * ut2;
		const float f3 = r3 * tt;
		w = f1 + f2 + f3;
		
		// Calculate point.
		x = (f1 * p1x + f2 * p2x + f3 * p3x) / w;
		y = (f1 * p1y + f2 * p2y + f3 * p3y) / w;
	}
	
	/** Calculate the normal at the given t value for a rational quadratic bezier curve defined by
	  * two vertices (`p1` and `p3`) and a control point (`p2`), and the corresponding ratios/weights. */
	void eval_normal(
		const float p1x, const float p1y, const float p2x, const float p2y, const float p3x, const float p3y,
		const float r1, const float r2, const float r3,
		const float t, float &out normal_x, float &out normal_y)
	{
		// Calculate the point.
		const float u = 1 - t;
		const float tt = t*t;
		const float uu = u*u;
		const float ut2 = 2*u*t;
		
		const float f1 = r1 * uu;
		const float f2 = r2 * ut2;
		const float f3 = r3 * tt;
		const float basis = f1 + f2 + f3;
		
		// Calculate normal.
		const float d2 = 2*(
			+ u*(r2 - r1)
			- t*(r2 - r3));
		normal_x =
			(2*(u*(r2*p2y - r1*p1y) + t*(r3 * p3y - r2 * p2y))) / basis -
			(d2 * (r1 * p1y * uu + 2 * r2 * p2y * t * u + r3 * p3y * tt)) / (basis * basis);
		normal_y = -(
			(2*(u*(r2*p2x - r1*p1x) + t*(r3 * p3x - r2 * p2x))) / basis -
			(d2 * (r1 * p1x * uu + 2 * r2 * p2x * t * u + r3 * p3x * tt)) / (basis * basis));
		
		const float length = sqrt(normal_x * normal_x + normal_y * normal_y);
		if(length != 0)
		{
			normal_x /= length;
			normal_y /= length;
		}
	}
	
}
