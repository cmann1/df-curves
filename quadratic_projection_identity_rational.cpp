namespace QuadraticBezier
{
	
	/** Calculate and return the A and C values for a rational quadratic curve based on the start and end points, a B value, u value, and ratio
	  * which can be obtained using `QuadraticBezier::calc_abc_ratio`. */
	void calc_abc(
		const float p1x, const float p1y, const float r1, const float p3x, const float p3y, const float r3,
		const float u, const float ratio, const float bx, const float by, const float br,
		float &out ax, float &out ay, float &out ar, float &out cx, float &out cy, float &out cr)
	{
		cr = u*r1 + (1 - u)*r3;
		cx = (u*p1x*r1 + (1 - u)*p3x*r3);
		cy = (u*p1y*r1 + (1 - u)*p3y*r3);
		ar = br + (br - cr)/ratio;
		ax = (bx*br + (bx*br - cx)/ratio)/ar;
		ay = (by*br + (by*br - cy)/ratio)/ar;
		cx /= cr;
		cy /= cr;
	}
	
}
