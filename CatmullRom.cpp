namespace CatmullRom
{
	
	/** Calculate the position and normal at the given t value.
	 * @param normalise If false the returned normal values will not be normalised. */
	void eval(
		const float p1x, const float p1y, const float p2x, const float p2y,
		const float p3x, const float p3y, const float p4x, const float p4y,
		const float tension,
		const float t, float &out x, float &out y, float &out normal_x, float &out normal_y,
		const bool normalise=true)
	{
		const float st = tension * 2;
		
		// Calculate the point.
		const float t2 = t * t;
		const float t3 = t2 * t;
		
		const float dv1x = (p3x - p1x) / st;
		const float dv1y = (p3y - p1y) / st;
		const float dv2x = (p4x - p2x) / st;
		const float dv2y = (p4y - p2y) / st;
		
		const float c0 = 2 * t3 - 3 * t2 + 1;
		const float c1 = t3 - 2 * t2 + t;
		const float c2 = -2 * t3 + 3 * t2;
		const float c3 = t3 - t2;
		x = c0 * p2x + c1 * dv1x + c2 * p3x + c3 * dv2x;
		y = c0 * p2y + c1 * dv1y + c2 * p3y + c3 * dv2y;
		
		// Calculate the normal.
		normal_x =
			(3 * t2 - 4 * t + 1) * dv1y +
			(3 * t2 - 2 * t) * dv2y +
			p2y * (6 * t2 - 6 * t) + p3y * (6 * t - 6 * t2);
		normal_y = -(
			(3 * t2 - 4 * t + 1) * dv1x +
			(3 * t2 - 2 * t) * dv2x +
			p2x * (6 * t2 - 6 * t) + p3x * (6 * t - 6 * t2));
		
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
	
	void eval_point(
		const float p1x, const float p1y, const float p2x, const float p2y,
		const float p3x, const float p3y, const float p4x, const float p4y,
		const float tension,
		const float t, float &out x, float &out y)
	{
		const float st = tension * 2;
		
		// Calculate the point.
		const float t2 = t * t;
		const float t3 = t2 * t;
		
		const float dv1x = (p3x - p1x) / st;
		const float dv1y = (p3y - p1y) / st;
		const float dv2x = (p4x - p2x) / st;
		const float dv2y = (p4y - p2y) / st;
		
		const float c0 = 2 * t3 - 3 * t2 + 1;
		const float c1 = t3 - 2 * t2 + t;
		const float c2 = -2 * t3 + 3 * t2;
		const float c3 = t3 - t2;
		x = c0 * p2x + c1 * dv1x + c2 * p3x + c3 * dv2x;
		y = c0 * p2y + c1 * dv1y + c2 * p3y + c3 * dv2y;
	}
	
	void eval_normal(
		const float p1x, const float p1y, const float p2x, const float p2y,
		const float p3x, const float p3y, const float p4x, const float p4y,
		const float tension,
		const float t, float &out normal_x, float &out normal_y,
		const bool normalise=true)
	{
		const float st = tension * 2;
		
		// Calculate the point.
		const float t2 = t * t;
		const float t3 = t2 * t;
		
		const float dv1x = (p3x - p1x) / st;
		const float dv1y = (p3y - p1y) / st;
		const float dv2x = (p4x - p2x) / st;
		const float dv2y = (p4y - p2y) / st;
		
		const float c0 = 2 * t3 - 3 * t2 + 1;
		const float c1 = t3 - 2 * t2 + t;
		const float c2 = -2 * t3 + 3 * t2;
		const float c3 = t3 - t2;
		
		// Calculate the normal.
		normal_x =
			(3 * t2 - 4 * t + 1) * dv1y +
			(3 * t2 - 2 * t) * dv2y +
			p2y * (6 * t2 - 6 * t) + p3y * (6 * t - 6 * t2);
		normal_y = -(
			(3 * t2 - 4 * t + 1) * dv1x +
			(3 * t2 - 2 * t) * dv2x +
			p2x * (6 * t2 - 6 * t) + p3x * (6 * t - 6 * t2));
		
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
