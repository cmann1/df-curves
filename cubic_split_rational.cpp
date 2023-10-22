namespace CubicBezier
{
	
	/** Split a rational cubic bezier curve at the given t value into two smaller curves.
	  * The start, end, values are not returned as they are the same as p1 and p3, and the middle values are shared by both sides. */
	void split(
		const float p1x, const float p1y, const float p2x, const float p2y, const float p3x, const float p3y, const float p4x, const float p4y,
		const float r1, const float r2, const float r3, const float r4,
		const float t,
		float &out a_p2x, float &out a_p2y, float &out a_p3x, float &out a_p3y, float &out m_x, float &out m_y,
		float &out b_p2x, float &out b_p2y, float &out b_p3x, float &out b_p3y,
		float &out a_r2, float &out a_r3, float &out m_r, float &out b_r2, float &out b_r3)
	{
		const float tt = t*t;
		const float u2 = (t - 1)*(t - 1);
		
		a_r2 = t*r2 - (t - 1)*r1;
		a_p2x = (t*p2x*r2 - (t - 1)*p1x*r1)/a_r2;
		a_p2y = (t*p2y*r2 - (t - 1)*p1y*r1)/a_r2;
		a_r3 = tt*r3 - 2*t*(t - 1)*r2 + u2*r1;
		a_p3x = (tt*p3x*r3 - 2*t*(t - 1)*p2x*r2 + u2*p1x*r1)/a_r3;
		a_p3y = (tt*p3y*r3 - 2*t*(t - 1)*p2y*r2 + u2*p1y*r1)/a_r3;
		
		m_r = tt*t*r4 - 3*tt*(t - 1)*r3 + 3*t*u2*r2 - u2*(t - 1)*r1;
		m_x = (tt*t*p4x*r4 - 3*tt*(t - 1)*p3x*r3 + 3*t*u2*p2x*r2 - u2*(t - 1)*p1x*r1)/m_r;
		m_y = (tt*t*p4y*r4 - 3*tt*(t - 1)*p3y*r3 + 3*t*u2*p2y*r2 - u2*(t - 1)*p1y*r1)/m_r;
		
		b_r2 = tt*r4 - 2*t*(t - 1)*r3 + u2*r2;
		b_p2x = (tt*p4x*r4 - 2*t*(t - 1)*p3x*r3 + u2*p2x*r2)/b_r2;
		b_p2y = (tt*p4y*r4 - 2*t*(t - 1)*p3y*r3 + u2*p2y*r2)/b_r2;
		b_r3 = t*r4 - (t - 1)*r3;
		b_p3x = (t*p4x*r4 - (t - 1)*p3x*r3)/b_r3;
		b_p3y = (t*p4y*r4 - (t - 1)*p3y*r3)/b_r3;
	}
	
}
