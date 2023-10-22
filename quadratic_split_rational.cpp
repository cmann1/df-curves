namespace QuadraticBezier
{
	
	/** Split a rational quadratic bezier curve at the given t value into two smaller curves.
	  * The start, end, values are not returned as they are the same as p1 and p3, and the middle values are shared by both sides. */
	void split(
		const float p1x, const float p1y, const float p2x, const float p2y, const float p3x, const float p3y,
		const float r1, const float r2, const float r3,
		const float t,
		float &out a_p2x, float &out a_p2y, float &out m_x, float &out m_y, float &out b_p2x, float &out b_p2y,
		float &out a_r2, float &out m_r, float &out b_r2)
	{
		a_r2 = t*r2 - (t - 1)*r1;
		a_p2x = (t*p2x*r2 - (t - 1)*p1x*r1)/a_r2;
		a_p2y = (t*p2y*r2 - (t - 1)*p1y*r1)/a_r2;
		
		m_r = t*t*r3 - 2*t*(t - 1)*r2 + (t - 1)*(t - 1)*r1;
		m_x = (t*t*p3x*r3 - 2*t*(t - 1)*p2x*r2 + (t - 1)*(t - 1)*p1x*r1)/m_r;
		m_y = (t*t*p3y*r3 - 2*t*(t - 1)*p2y*r2 + (t - 1)*(t - 1)*p1y*r1)/m_r;
		
		b_r2 = t*r3 - (t - 1)*r2;
		b_p2x = (t*p3x*r3 - (t - 1)*p2x*r2)/b_r2;
		b_p2y = (t*p3y*r3 - (t - 1)*p2y*r2)/b_r2;
	}
	
}
