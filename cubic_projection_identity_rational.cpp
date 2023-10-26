namespace CubicBezier
{
	
	/** Calculates and returns the tangent/e1/e2 values for the rational cubic curve based on the curve points, B value, t value, u value, and ratio
	  * which can be obtained using `QuadraticBezier::calc_abc_ratio`.
	  * Returned values are in homogeneous space, i.e not divided by the ratio before being returned. */
	void calc_abc_tangent(
		const float p1x, const float p1y, const float p1r, const float p2x, const float p2y, const float p2r,
		const float p3x, const float p3y, const float p3r, const float p4x, const float p4y, const float p4r,
		const float t, const float u, const float ratio, const float bx, const float by, const float br,
		float &out e1x, float &out e1y, float &out e1r, float &out e2x, float &out e2y, float &out e2r)
	{
		const float bxr = bx*br;
		const float byr = by*br;
		
		const float cr = u*p1r + (1 - u)*p4r;
		const float cx = u*p1x*p1r + (1 - u)*p4x*p4r;
		const float cy = u*p1y*p1r + (1 - u)*p4y*p4r;
		const float ar = br + (br - cr)/ratio;
		const float ax = bxr + (bxr - cx)/ratio;
		const float ay = byr + (byr - cy)/ratio;
		
		const float it = 1 - t;
		
		const float v1r = p1r*it + p2r*t;
		const float v1x = p1x*p1r*it + p2x*p2r*t;
		const float v1y = p1y*p1r*it + p2y*p2r*t;
		const float v2r = p3r*it + p4r*t;
		const float v2x = p3x*p3r*it + p4x*p4r*t;
		const float v2y = p3y*p3r*it + p4y*p4r*t;
		
		e1r = it*v1r + ar*t - br;
		e1x = it*v1x + ax*t - bxr;
		e1y = it*v1y + ay*t - byr;
		e2r = it*ar + v2r*t - br;
		e2x = it*ax + v2x*t - bxr;
		e2y = it*ay + v2y*t - byr;
	}
	
	/** Calculate control points based on the given start, end, and tangent values. */
	void calc_from_abc_tangent(
		const float p1x, const float p1y, const float p1r, const float p4x, const float p4y, const float p4r,
		const float e1x, const float e1y, const float e1r, const float e2x, const float e2y, const float e2r,
		const float t, const float u, const float ratio, const float bx, const float by, const float br,
		float &out p2x, float &out p2y, float &out p2r, float &out p3x, float &out p3y, float &out p3r)
	{
		const float bxr = bx*br;
		const float byr = by*br;
		
		const float cr = u*p1r + (1 - u)*p4r;
		const float cx = (u*p1x*p1r + (1 - u)*p4x*p4r);
		const float cy = (u*p1y*p1r + (1 - u)*p4y*p4r);
		const float ar = br + (br - cr)/ratio;
		const float ax = bxr + (bxr - cx)/ratio;
		const float ay = byr + (byr - cy)/ratio;
		
		const float it = 1 - t;
		
		const float v1r = (br + e1r - t*ar)/it;
		const float v1x = (bxr + e1x - t*ax)/it;
		const float v1y = (byr + e1y - t*ay)/it;
		const float v2r = (br + e2r - it*ar)/t;
		const float v2x = (bxr + e2x - it*ax)/t;
		const float v2y = (byr + e2y - it*ay)/t;
		
		p2r = (v1r - it*p1r)/t;
		p2x = ((v1x - it*p1x*p1r)/t)/p2r;
		p2y = ((v1y - it*p1y*p1r)/t)/p2r;
		p3r = (v2r - t*p4r)/it;
		p3x = ((v2x - t*p4x*p4r)/it)/p3r;
		p3y = ((v2y - t*p4y*p4r)/it)/p3r;
	}
	
}
