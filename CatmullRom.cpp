namespace CatmullRom
{
	
	/// Converts four points and a tension value representing a CatmullRom curve to two vertices and two
	/// (relative) control points representing a cubic Bezier curve.
	void to_cubic_bezier(
		const float p1x, const float p1y, const float p2x, const float p2y,
		const float p3x, const float p3y, const float p4x, const float p4y, const float tension,
		float &out cb_p1x, float &out cb_p1y, float &out cb_p2x, float &out cb_p2y,
		float &out cb_p3x, float &out cb_p3y, float &out cb_p4x, float &out cb_p4y)
	{
		const float a = 6 * tension;
		cb_p1x = p2x;
		cb_p1y = p2y;
		cb_p2x = (p3x - p1x) / a;
		cb_p2y = (p3y - p1y) / a;
		cb_p3x = -(p4x - p2x) / a;
		cb_p3y = -(p4y - p2y) / a;
		cb_p4x = p3x;
		cb_p4y = p3y;
	}
	
}
