namespace QuadraticBezier
{
	
	// -- Eval --
	
	/** Calculate the position and normal at the given t value for a non-rational quadratic bezier curve defined by
	  * two vertices (`p1` and `p3`) and a control point (`p2`).
	  * @param normalise If false the returned normal values will not be normalised. */
	void eval(
		const float p1x, const float p1y, const float p2x, const float p2y, const float p3x, const float p3y,
		const float t, float &out x, float &out y, float &out normal_x, float &out normal_y,
		const bool normalise=true)
	{
		// Calculate the point.
		const float u = 1 - t;
		const float tt = t*t;
		const float uu = u*u;
		const float ut2 = 2*u*t;
		
		x = uu*p1x + ut2*p2x + tt*p3x;
		y = uu*p1y + ut2*p2y + tt*p3y;
		normal_x =  2*(u*(p2y - p1y) + t*(p3y - p2y));
		normal_y = -2*(u*(p2x - p1x) + t*(p3x - p2x));
		
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
	  * two vertices (`p1` and `p3`) and a control point (`p2`). */
	void eval_point(
		const float p1x, const float p1y, const float p2x, const float p2y, const float p3x, const float p3y,
		const float t, float &out x, float &out y)
	{
		// Calculate the point.
		const float u = 1 - t;
		const float tt = t*t;
		const float uu = u*u;
		const float ut2 = 2*u*t;
		
		x = uu * p1x + ut2 * p2x + tt * p3x;
		y = uu * p1y + ut2 * p2y + tt * p3y;
	}
	
	/** Calculate the normal at the given t value for a non-rational quadratic bezier curve defined by
	  * two vertices (`p1` and `p3`) and a control point (`p2`). */
	void eval_normal(
		const float p1x, const float p1y, const float p2x, const float p2y, const float p3x, const float p3y,
		const float t, float &out normal_x, float &out normal_y, const bool normalise=true)
	{
		// Calculate the point.
		const float u = 1 - t;
		const float ut2 = 2*u*t;
		
		normal_x =  2 * (u * (p2y - p1y) + t * (p3y - p2y));
		normal_y = -2 * (u * (p2x - p1x) + t * (p3x - p2x));
		
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
	  * two vertices (`p1` and `p3`) and a control point (`p2`). */
	void bounding_box(
		const float p1x, const float p1y, const float p2x, const float p2y, const float p3x, const float p3y,
		float &out x1, float &out y1, float &out x2, float &out y2)
	{
		x1 = p1x < p3x ? p1x : p3x;
		y1 = p1y < p3y ? p1y : p3y;
		x2 = p3x > p1x ? p3x : p1x;
		y2 = p3y > p1y ? p3y : p1y;
		
		const float ax = 2*(p2x - p1x);
		const float ay = 2*(p2y - p1y);
		const float bx = 2*(p3x - p2x);
		const float by = 2*(p3y - p2y);
		
		// Calculate the x/y roots.
		const float tx = bx - ax != 0 ? -ax/(bx - ax) : -1;
		const float ty = by - ay != 0 ? -ay/(by - ay) : -1;
		
		if(tx > 0 && tx < 1)
		{
			const float u = 1 - tx;
			const float tt = tx*tx;
			const float uu = u*u;
			const float ut2 = 2*u*tx;
			
			const float x = uu * p1x + ut2 * p2x + tt * p3x;
			
			if(x < x1) x1 = x;
			if(x > x2) x2 = x;
		}
		
		if(ty > 0 && ty < 1)
		{
			const float u = 1 - ty;
			const float tt = ty*ty;
			const float uu = u*u;
			const float ut2 = 2*u*ty;
			
			const float y = uu * p1y + ut2 * p2y + tt * p3y;
			
			if(y < y1) y1 = y;
			if(y > y2) y2 = y;
		}
	}
	
}
