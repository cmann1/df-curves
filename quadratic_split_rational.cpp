namespace QuadraticBezier
{
	
	/** Split a rational quadratic bezier curve at the given t value into two smaller curves. */
	void split(
		const float p1x, const float p1y, const float p2x, const float p2y, const float p3x, const float p3y,
		const float r1, const float r2, const float r3,
		const float t,
		float &out a_p1x, float &out a_p1y, float &out a_p2x, float &out a_p2y, float &out a_p3x, float &out a_p3y,
		float &out b_p1x, float &out b_p1y, float &out b_p2x, float &out b_p2y, float &out b_p3x, float &out b_p3y,
		float &out a_r1, float &out a_r2, float &out a_r3,
		float &out b_r1, float &out b_r2, float &out b_r3)
	{
		a_r1 = r1;
		a_p1x = p1x;
		a_p1y = p1y;
		a_r2 = t*r2 - (t - 1)*r1;
		a_p2x = (t*p2x*r2 - (t - 1)*p1x*r1)/a_r2;
		a_p2y = (t*p2y*r2 - (t - 1)*p1y*r1)/a_r2;
		a_r3 = t*t*r3 - 2*t*(t - 1)*r2 + (t - 1)*(t - 1)*r1;
		a_p3x = (t*t*p3x*r3 - 2*t*(t - 1)*p2x*r2 + (t - 1)*(t - 1)*p1x*r1)/a_r3;
		a_p3y = (t*t*p3y*r3 - 2*t*(t - 1)*p2y*r2 + (t - 1)*(t - 1)*p1y*r1)/a_r3;
		
		b_r1 = a_r3;
		b_p1x = a_p3x;
		b_p1y = a_p3y;
		b_r2 = t*r3 - (t - 1)*r2;
		b_p2x = (t*p3x*r3 - (t - 1)*p2x*r2)/b_r2;
		b_p2y = (t*p3y*r3 - (t - 1)*p2y*r2)/b_r2;
		b_r3 = r3;
		b_p3x = p3x;
		b_p3y = p3y;
	}
	
}
