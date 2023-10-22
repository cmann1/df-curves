namespace CubicBezier
{
	
	/** Split a cubic bezier curve at the given t value into two smaller curves.
	  * The start, end, values are not returned as they are the same as p1 and p3, and the middle values are shared by both sides. */
	void split(
		const float p1x, const float p1y, const float p2x, const float p2y, const float p3x, const float p3y, const float p4x, const float p4y,
		const float t,
		float &out a_p2x, float &out a_p2y, float &out a_p3x, float &out a_p3y, float &out m_x, float &out m_y,
		float &out b_p2x, float &out b_p2y, float &out b_p3x, float &out b_p3y)
	{
		const float tt = t*t;
		const float u2 = (t - 1)*(t - 1);
		
		a_p2x = t*p2x - (t - 1)*p1x;
		a_p2y = t*p2y - (t - 1)*p1y;
		a_p3x = tt*p3x - 2*t*(t - 1)*p2x + u2*p1x;
		a_p3y = tt*p3y - 2*t*(t - 1)*p2y + u2*p1y;
		
		m_x = tt*t*p4x - 3*tt*(t - 1)*p3x + 3*t*u2*p2x - u2*(t - 1)*p1x;
		m_y = tt*t*p4y - 3*tt*(t - 1)*p3y + 3*t*u2*p2y - u2*(t - 1)*p1y;
		
		b_p2x = tt*p4x - 2*t*(t - 1)*p3x + u2*p2x;
		b_p2y = tt*p4y - 2*t*(t - 1)*p3y + u2*p2y;
		b_p3x = t*p4x - (t - 1)*p3x;
		b_p3y = t*p4y - (t - 1)*p3y;
	}
	
}

