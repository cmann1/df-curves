namespace QuadraticBezier
{
	
	/** Split a quadratic bezier curve at the given t value into two smaller curves.
	  * The start, end, values are not returned as they are the same as p1 and p3, and the middle values are shared by both sides. */
	void split(
		const float p1x, const float p1y, const float p2x, const float p2y, const float p3x, const float p3y,
		const float t,
		float &out a_p2x, float &out a_p2y, float &out m_x, float &out m_y, float &out b_p2x, float &out b_p2y)
	{
		a_p2x = t*p2x - (t - 1)*p1x;
		a_p2y = t*p2y - (t - 1)*p1y;
		
		m_x = t*t*p3x - 2*t*(t - 1)*p2x + (t - 1)*(t - 1)*p1x;
		m_y = t*t*p3y - 2*t*(t - 1)*p2y + (t - 1)*(t - 1)*p1y;
		
		b_p2x = t*p3x - (t - 1)*p2x;
		b_p2y = t*p3y - (t - 1)*p2y;
	}
	
}
