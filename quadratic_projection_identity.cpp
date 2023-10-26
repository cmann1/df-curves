namespace QuadraticBezier
{
	
	/** Calculate and return the abc ratio and u value based on the given t value.
	  * https://pomax.github.io/bezierinfo/#abc */
	void calc_abc_ratio(const float t, float &out u, float &out ratio)
	{
		const float it2 = (1 - t)*(1 - t);
		const float den = t*t + it2;
		u = it2/den;
		ratio = abs((den - 1) / den);
	}
	
	
	/** Calculate and return the A and C values based on the start and end points, B value, u value, and ratio
	  * which can be obtained using `QuadraticBezier::calc_abc_ratio`. */
	void calc_abc(
		const float p1x, const float p1y, const float p3x, const float p3y,
		const float u, const float ratio, const float bx, const float by,
		float &out ax, float &out ay, float &out cx, float &out cy)
	{
		cx = u*p1x + (1 - u)*p3x;
		cy = u*p1y + (1 - u)*p3y;
		ax = bx + (bx - cx)/ratio;
		ay = by + (by - cy)/ratio;
	}
	
}
