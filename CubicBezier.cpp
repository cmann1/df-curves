namespace CubicBezier
{
	
	/** Calculate the bounding box of a non-rational quadratic bezier curve defined by
	  * two vertices (`p1` and `p4`) and two control point (`p2` and `p3`). */
	void bounding_box(
		const float p1x, const float p1y, const float p2x, const float p2y,
		const float p3x, const float p3y, const float p4x, const float p4y,
		float &out x1, float &out y1, float &out x2, float &out y2)
	{
		x1 = p1x < p4x ? p1x : p1x;
		y1 = p1y < p4y ? p1y : p1y;
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
	
	/** Calculate the bounding box of a non-rational quadratic bezier curve defined by
	  * two vertices (`p1` and `p4`), two control point (`p2` and `p3`), and the corresponding ratios/weights,
	  * using the Newton method.
	  * 
	  * Thanks to Skyhawk for figuring this out:
	  * https://discord.com/channels/83037671227658240/342175833089245184/1158941595228450867
	  * https://www.desmos.com/calculator/mxt9wq6kzn
	  * 
	  * @param samples How many points along the curve to sample to find the roots. Increase to increase the accuracy of the resulting boudning box.
	  * @param padding Allows expanding the resulting bounding box on all four side to account for possible inaccuracies. */
	void bounding_box(
		const float p1x, const float p1y, const float p2x, const float p2y,
		const float p3x, const float p3y, const float p4x, const float p4y,
		const float r1, const float r2, const float r3, const float r4,
		float &out x1, float &out y1, float &out x2, float &out y2,
		const int samples=6, const float padding=0.5)
	{
		x1 = (p1x < p4x ? p1x : p1x) - padding;
		y1 = (p1y < p4y ? p1y : p1y) - padding;
		x2 = (p4x > p1x ? p4x : p1x) + padding;
		y2 = (p4y > p1y ? p4y : p1y) + padding;
		
		for(int i = 0; i < samples; i++)
		{
			const float t = float(i) / (samples - 1);
			
			const float t2 = t * t;
			const float t3 = t2 * t;
			const float t4 = t3 * t;
			
			const float r10x = r2*r1*(p2x - p1x);
			const float r20x = r3*r1*(p3x - p1x);
			const float r21x = r3*r2*(p3x - p2x);
			const float r30x = r4*r1*(p4x - p1x);
			const float r31x = r4*r2*(p4x - p2x);
			const float ax = 3*r21x - 2*r20x - 2*r31x + r4*r3*(p4x - p3x) + r10x + r30x;
			const float bx = 3*(r20x - r21x) - 2*r10x - r30x + r31x;
			const float cx = 6*(r10x - r20x) + 3*r21x + r30x;
			const float dx = r20x - 2*r10x;
			const float rdx = 3*(t2*(ax*t2 + cx) + 2*t*(bx*t2 + dx) + r10x);
			const float rd2x = 6*(2*ax*t3 + 3*bx*t2 + cx*t + dx);
			
			const float ntx = rd2x != 0 ? -rdx / rd2x + t : 1;
			if(ntx >= 0 && ntx <= 1)
			{
				const float u = 1 - ntx;
				const float tt = ntx * ntx;
				const float tt3 = tt * ntx;
				const float uu = u * u;
				const float uuu = uu * u;
				const float f0 = uuu * r1;
				const float f1 = 3 * uu * ntx * r2;
				const float f2 = 3 * u * tt * r3;
				const float f3 = tt3 * r4;
				const float basis = f0 + f1 + f2 + f3;
				const float x = (f0 * p1x + f1 * p2x + f2 * p3x + f3 * p4x) / basis;
				
				if(x - padding < x1) x1 = x - padding;
				if(x + padding > x2) x2 = x + padding;
			}
			
			const float r10y = r2*r1*(p2y - p1y);
			const float r20y = r3*r1*(p3y - p1y);
			const float r21y = r3*r2*(p3y - p2y);
			const float r30y = r4*r1*(p4y - p1y);
			const float r31y = r4*r2*(p4y - p2y);
			const float ay = 3*r21y - 2*r20y - 2*r31y + r4*r3*(p4y - p3y) + r10y + r30y;
			const float by = 3*(r20y - r21y) - 2*r10y - r30y + r31y;
			const float cy = 6*(r10y - r20y) + 3*r21y + r30y;
			const float dy = r20y - 2*r10y;
			const float rdy = 3*(t2*(ay*t2 + cy) + 2*t*(by*t2 + dy) + r10y);
			const float rd2y = 6*(2*ay*t3 + 3*by*t2 + cy*t + dy);
			
			const float nty = rd2y != 0 ? -rdy / rd2y + t : 1;
			if(nty >= 0 && nty <= 1)
			{
				const float u = 1 - nty;
				const float tt = nty * nty;
				const float tt3 = tt * nty;
				const float uu = u * u;
				const float uuu = uu * u;
				const float f0 = uuu * r1;
				const float f1 = 3 * uu * nty * r2;
				const float f2 = 3 * u * tt * r3;
				const float f3 = tt3 * r4;
				const float basis = f0 + f1 + f2 + f3;
				const float y = (f0 * p1y + f1 * p2y + f2 * p3y + f3 * p4y) / basis;
				
				if(y - padding < y1) y1 = y - padding;
				if(y + padding > y2) y2 = y + padding;
			}
		}
	}
	
}
