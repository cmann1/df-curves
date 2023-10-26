namespace CubicBezier
{
	
	/** Calculate and return the abc ratio and u value based on the given t value.
	  * https://pomax.github.io/bezierinfo/#abc */
	void calc_abc_ratio(const float t, float &out u, float &out ratio)
	{
		const float it3 = (1 - t)*(1 - t)*(1 - t);
		const float den = t*t*t + it3;
		u = it3/den;
		ratio = abs((den - 1) / den);
	}
	
	/** Calculate the tangent/e1/e2 values for the cubic curve based on the curve points, B value, t value, u value, and ratio
	  * which can be obtained using `QuadraticBezier::calc_abc_ratio`. */
	void calc_abc_tangent(
		const float p1x, const float p1y, const float p2x, const float p2y, const float p3x, const float p3y, const float p4x, const float p4y,
		const float t, const float u, const float ratio, const float bx, const float by,
		float &out e1x, float &out e1y, float &out e2x, float &out e2y)
	{
		const float cx = u*p1x + (1 - u)*p4x;
		const float cy = u*p1y + (1 - u)*p4y;
		const float ax = bx + (bx - cx)/ratio;
		const float ay = by + (by - cy)/ratio;
		
		const float it = 1 - t;
		
		const float v1x = p1x*it + p2x*t;
		const float v1y = p1y*it + p2y*t;
		const float v2x = p3x*it + p4x*t;
		const float v2y = p3y*it + p4y*t;
		e1x = it*v1x + ax*t - bx;
		e1y = it*v1y + ay*t - by;
		e2x = it*ax + v2x*t - bx;
		e2y = it*ay + v2y*t - by;
	}
	
	/** Calculate control points based on the given start, end, and tangent values. */
	void calc_from_abc_tangent(
		const float p1x, const float p1y, const float p4x, const float p4y,
		const float e1x, const float e1y, const float e2x, const float e2y,
		const float t, const float u, const float ratio, const float bx, const float by,
		float &out p2x, float &out p2y, float &out p3x, float &out p3y)
	{
		const float it = 1 - t;
		
		const float cx = u*p1x + (1 - u)*p4x;
		const float cy = u*p1y + (1 - u)*p4y;
		const float ax = bx + (bx - cx)/ratio;
		const float ay = by + (by - cy)/ratio;
		
		const float v1x = (bx + e1x - t*ax)/it;
		const float v1y = (by + e1y - t*ay)/it;
		const float v2x = (bx + e2x - it*ax)/t;
		const float v2y = (by + e2y - it*ay)/t;
		
		p2x = (v1x - it*p1x)/t;
		p2y = (v1y - it*p1y)/t;
		p3x = (v2x - t*p4x)/it;
		p3y = (v2y - t*p4y)/it;
	}
	
}
