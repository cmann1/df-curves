namespace CubicBezier
{
	
	/** Split a rational cubic bezier curve at the given t value into two smaller curves. */
	void split(
		const float p1x, const float p1y, const float p2x, const float p2y, const float p3x, const float p3y, const float p4x, const float p4y,
		const float r1, const float r2, const float r3, const float r4,
		const float t,
		float &out a_p1x, float &out a_p1y, float &out a_p2x, float &out a_p2y, float &out a_p3x, float &out a_p3y, float &out a_p4x, float &out a_p4y,
		float &out b_p1x, float &out b_p1y, float &out b_p2x, float &out b_p2y, float &out b_p3x, float &out b_p3y, float &out b_p4x, float &out b_p4y,
		float &out a_r1, float &out a_r2, float &out a_r3, float &out a_r4,
		float &out b_r1, float &out b_r2, float &out b_r3, float &out b_r4)
	{
		const float tt = t*t;
		const float u2 = (t - 1)*(t - 1);
		
		a_r1 = r1;
		a_p1x = p1x;
		a_p1y = p1y;
		a_r2 = t*r2 - (t - 1)*r1;
		a_p2x = (t*p2x*r2 - (t - 1)*p1x*r1)/a_r2;
		a_p2y = (t*p2y*r2 - (t - 1)*p1y*r1)/a_r2;
		a_r3 = tt*r3 - 2*t*(t - 1)*r2 + u2*r1;
		a_p3x = (tt*p3x*r3 - 2*t*(t - 1)*p2x*r2 + u2*p1x*r1)/a_r3;
		a_p3y = (tt*p3y*r3 - 2*t*(t - 1)*p2y*r2 + u2*p1y*r1)/a_r3;
		a_r4 = tt*t*r4 - 3*tt*(t - 1)*r3 + 3*t*u2*r2 - u2*(t - 1)*r1;
		a_p4x = (tt*t*p4x*r4 - 3*tt*(t - 1)*p3x*r3 + 3*t*u2*p2x*r2 - u2*(t - 1)*p1x*r1)/a_r4;
		a_p4y = (tt*t*p4y*r4 - 3*tt*(t - 1)*p3y*r3 + 3*t*u2*p2y*r2 - u2*(t - 1)*p1y*r1)/a_r4;
		
		b_r1 = a_r4;
		b_p1x = a_p4x;
		b_p1y = a_p4y;
		b_r2 = tt*r4 - 2*t*(t - 1)*r3 + u2*r2;
		b_p2x = (tt*p4x*r4 - 2*t*(t - 1)*p3x*r3 + u2*p2x*r2)/b_r2;
		b_p2y = (tt*p4y*r4 - 2*t*(t - 1)*p3y*r3 + u2*p2y*r2)/b_r2;
		b_r3 = t*r4 - (t - 1)*r3;
		b_p3x = (t*p4x*r4 - (t - 1)*p3x*r3)/b_r3;
		b_p3y = (t*p4y*r4 - (t - 1)*p3y*r3)/b_r3;
		b_r4 = r4;
		b_p4x = p4x;
		b_p4y = p4y;
	}
	
}
