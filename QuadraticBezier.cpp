namespace QuadraticBezier
{
	
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
	
	/** Calculate the bounding box of a rational quadratic bezier curve defined by
	  * two vertices (`p1` and `p3`), a control point (`p2`), and the corresponding ratios/weights.
	  * 
	  * Once again thanks to Skyhawk for figuring this out:
	  * https://discord.com/channels/83037671227658240/342175833089245184/1160156912415932469 */
	void bounding_box(
		const float p1x, const float p1y, const float p2x, const float p2y, const float p3x, const float p3y,
		const float r1, const float r2, const float r3,
		float &out x1, float &out y1, float &out x2, float &out y2)
	{
		x1 = p1x < p3x ? p1x : p3x;
		y1 = p1y < p3y ? p1y : p3y;
		x2 = p3x > p1x ? p3x : p1x;
		y2 = p3y > p1y ? p3y : p1y;
		
		const float ax = 2*(r1*r2*(p2x - p1x) + r2*r3*(p3x - p2x) + r1*r3*(p1x - p3x));
		const float ay = 2*(r1*r2*(p2y - p1y) + r2*r3*(p3y - p2y) + r1*r3*(p1y - p3y));
		const float bx = 2*(2*r1*r2*(p1x - p2x) + r1*r3*(p3x - p1x));
		const float by = 2*(2*r1*r2*(p1y - p2y) + r1*r3*(p3y - p1y));
		const float cx = 2*r1*r2*(p2x - p1x);
		const float cy = 2*r1*r2*(p2y - p1y);
		
		// Calculate the x and y roots by plugging the a, b, and c coefficients into the quadratic formula.
		const float dsc_x = sqrt(bx*bx - 4*ax*cx);
		const float dsc_y = sqrt(by*by - 4*ay*cy);
		const float t1x = abs(ax) > 0.01 ? clamp01((-bx + dsc_x)/(2*ax)) : abs(bx) > 0.01 ? -cx/bx : -1;
		const float t2x = abs(ax) > 0.01 ? clamp01((-bx - dsc_x)/(2*ax)) : abs(bx) > 0.01 ? -cx/bx : -1;
		const float t1y = abs(ay) > 0.01 ? clamp01((-by + dsc_y)/(2*ay)) : abs(by) > 0.01 ? -cy/by : -1;
		const float t2y = abs(ay) > 0.01 ? clamp01((-by - dsc_y)/(2*ay)) : abs(by) > 0.01 ? -cy/by : -1;
		
		if(t1x > 0 && t1x < 1)
		{
			const float u = 1 - t1x;
			const float tt = t1x * t1x;
			const float uu = u * u;
			const float ut2 = 2 * u * t1x;
			const float f1 = r1 * uu;
			const float f2 = r2 * ut2;
			const float f3 = r3 * tt;
			const float basis = f1 + f2 + f3;
			const float x = (f1 * p1x + f2 * p2x + f3 * p3x) / basis;
			
			if(x < x1) x1 = x;
			if(x > x2) x2 = x;
		}
		
		if(t2x > 0 && t2x < 1)
		{
			const float u = 1 - t2x;
			const float tt = t2x * t2x;
			const float uu = u * u;
			const float ut2 = 2 * u * t2x;
			const float f1 = r1 * uu;
			const float f2 = r2 * ut2;
			const float f3 = r3 * tt;
			const float basis = f1 + f2 + f3;
			const float x = (f1 * p1x + f2 * p2x + f3 * p3x) / basis;
			
			if(x < x1) x1 = x;
			if(x > x2) x2 = x;
		}
		
		if(t1y > 0 && t1y < 1)
		{
			const float u = 1 - t1y;
			const float tt = t1y * t1y;
			const float uu = u * u;
			const float ut2 = 2 * u * t1y;
			const float f1 = r1 * uu;
			const float f2 = r2 * ut2;
			const float f3 = r3 * tt;
			const float basis = f1 + f2 + f3;
			const float y = (f1 * p1y + f2 * p2y + f3 * p3y) / basis;
			
			if(y < y1) y1 = y;
			if(y > y2) y2 = y;
		}
		
		if(t2y > 0 && t2y < 1)
		{
			const float u = 1 - t2y;
			const float tt = t2y * t2y;
			const float uu = u * u;
			const float ut2 = 2 * u * t2y;
			const float f1 = r1 * uu;
			const float f2 = r2 * ut2;
			const float f3 = r3 * tt;
			const float basis = f1 + f2 + f3;
			const float y = (f1 * p1y + f2 * p2y + f3 * p3y) / basis;
			
			if(y < y1) y1 = y;
			if(y > y2) y2 = y;
		}
	}
	
}
