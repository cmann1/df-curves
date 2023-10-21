namespace QuadraticBezier
{
	
	/** Split a quadratic bezier curve at the given t value into two smaller curves. */
	void split(
		const float p1x, const float p1y, const float p2x, const float p2y, const float p3x, const float p3y,
		const float t,
		float &out a_p1x, float &out a_p1y, float &out a_p2x, float &out a_p2y, float &out a_p3x, float &out a_p3y,
		float &out b_p1x, float &out b_p1y, float &out b_p2x, float &out b_p2y, float &out b_p3x, float &out b_p3y)
	{
		a_p1x = p1x;
		a_p1y = p1y;
		a_p2x = t*p2x - (t - 1)*p1x;
		a_p2y = t*p2y - (t - 1)*p1y;
		a_p3x = t*t*p3x - 2*t*(t - 1)*p2x + (t - 1)*(t - 1)*p1x;
		a_p3y = t*t*p3y - 2*t*(t - 1)*p2y + (t - 1)*(t - 1)*p1y;
		
		b_p1x = a_p3x;
		b_p1y = a_p3y;
		b_p2x = t*p3x - (t - 1)*p2x;
		b_p2y = t*p3y - (t - 1)*p2y;
		b_p3x = p3x;
		b_p3y = p3y;
	}
	
}
