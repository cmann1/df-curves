/// Contains "static" utility methods for woeking with curves.
namespace MultiCurve
{
	
	/// Converts four points and a tension value representing a CatmullRom curve to two vertices and two
	/// (relative) control points representing a CubicBezier curve.
	void catmull_rom_to_cubic_bezier(
		const float cr_p1_x, const float cr_p1_y, const float cr_p2_x, const float cr_p2_y,
		const float cr_p3_x, const float cr_p3_y, const float cr_p4_x, const float cr_p4_y, const float tension,
		float &out p1_x, float &out p1_y, float &out p2_x, float &out p2_y,
		float &out c1_x, float &out c1_y, float &out c2_x, float &out c2_y)
	{
		const float a = 6 * tension;
		p1_x = cr_p2_x;
		p1_y = cr_p2_y;
		p2_x = cr_p3_x;
		p2_y = cr_p3_y;
		c1_x = (cr_p3_x - cr_p1_x) / a;
		c1_y = (cr_p3_y - cr_p1_y) / a;
		c2_x = -(cr_p4_x - cr_p2_x) / a;
		c2_y = -(cr_p4_y - cr_p2_y) / a;
	}
	
	/// Return the bounding box enclosing the curve given the two vertices and a single control point
	/// defining a quadratic bezier curve.
	void bounding_box_quadratic_bezier(
		const float p1_x, const float p1_y, const float p2_x, const float p2_y,
		const float cp_x, const float cp_y,
		float &out x1, float &out y1, float &out x2, float &out y2)
	{
		x1 = p1_x < p2_x ? p1_x : p1_x;
		y1 = p1_y < p2_y ? p1_y : p1_y;
		x2 = p2_x > p1_x ? p2_x : p2_x;
		y2 = p2_y > p1_y ? p2_y : p2_y;
		
		const float a_x = 2 * (cp_x - p1_x);
		const float a_y = 2 * (cp_y - p1_y);
		const float b_x = 2 * (p2_x - cp_x);
		const float b_y = 2 * (p2_y - cp_y);
		
		// Calculate the x/y roots.
		const float t_x = b_x - a_x != 0 ? -a_x / (b_x - a_x) : -1;
		const float t_y = b_y - a_y != 0 ? -a_y / (b_y - a_y) : -1;
		
		if(t_x >= 0 && t_x <= 1)
		{
			const float u = 1 - t_x;
			const float tt = t_x * t_x;
			const float uu = u * u;
			const float ut2 = 2 * u * t_x;
			
			const float rx = uu * p1_x + ut2 * cp_x + tt * p2_x;
			
			if(rx < x1) x1 = rx;
			if(rx > x2) x2 = rx;
		}
		
		if(t_y >= 0 && t_y <= 1)
		{
			const float u = 1 - t_y;
			const float tt = t_y * t_y;
			const float uu = u * u;
			const float ut2 = 2 * u * t_y;
			
			const float ry = uu * p1_y + ut2 * cp_y + tt * p2_y;
			
			if(ry < y1) y1 = ry;
			if(ry > y2) y2 = ry;
		}
	}
	
	/// Return the bounding box enclosing the curve given the two vertices and two control points
	/// defining a cubic bezier curve.
	void bounding_box_cubic_bezier(
		const float p1_x, const float p1_y, const float p2_x, const float p2_y,
		const float cp1_x, const float cp1_y, const float cp2_x, const float cp2_y,
		float &out x1, float &out y1, float &out x2, float &out y2)
	{
		x1 = p1_x < p2_x ? p1_x : p1_x;
		y1 = p1_y < p2_y ? p1_y : p1_y;
		x2 = p2_x > p1_x ? p2_x : p2_x;
		y2 = p2_y > p1_y ? p2_y : p2_y;
		
		const float a_x = 3 * (-p1_x + 3 * cp1_x - 3 * cp2_x + p2_x);
		const float a_y = 3 * (-p1_y + 3 * cp1_y - 3 * cp2_y + p2_y);
		const float b_x = 6 * (p1_x - 2 * cp1_x + cp2_x);
		const float b_y = 6 * (p1_y - 2 * cp1_y + cp2_y);
		const float c_x = 3 * (cp1_x - p1_x);
		const float c_y = 3 * (cp1_y - p1_y);
		
		// Calculate the x and y roots by plugging the a, b, and c coefficients into the quadratic formula.
		const float dsc_x = sqrt(b_x * b_x - 4 * a_x * c_x);
		const float dsc_y = sqrt(b_y * b_y - 4 * a_y * c_y);
		const float t1_x = abs(a_x) > 0.01 ? (-b_x + dsc_x) / (2 * a_x) : abs(b_x) > 0.01 ? - c_x / b_x : -1;
		const float t2_x = abs(a_x) > 0.01 ? (-b_x - dsc_x) / (2 * a_x) : abs(b_x) > 0.01 ? - c_x / b_x : -1;
		const float t1_y = abs(a_y) > 0.01 ? (-b_y + dsc_y) / (2 * a_y) : abs(b_y) > 0.01 ? - c_y / b_y : -1;
		const float t2_y = abs(a_y) > 0.01 ? (-b_y - dsc_y) / (2 * a_y) : abs(b_y) > 0.01 ? - c_y / b_y : -1;
		
		if(t1_x >= 0 && t1_x <= 1)
		{
			const float u = 1 - t1_x;
			const float tt = t1_x * t1_x;
			const float tt3 = tt * t1_x;
			const float uu = u * u;
			const float uuu = uu * u;
			
			const float rx = uuu * p1_x + 3 * uu * t1_x * cp1_x + 3 * u * tt * cp2_x + tt3 * p2_x;
			
			if(rx < x1) x1 = rx;
			if(rx > x2) x2 = rx;
		}
		
		if(t2_x >= 0 && t2_x <= 1)
		{
			const float u = 1 - t2_x;
			const float tt = t2_x * t2_x;
			const float tt3 = tt * t2_x;
			const float uu = u * u;
			const float uuu = uu * u;
			const float rx = uuu * p1_x + 3 * uu * t2_x * cp1_x + 3 * u * tt * cp2_x + tt3 * p2_x;
			
			if(rx < x1) x1 = rx;
			if(rx > x2) x2 = rx;
		}
		
		if(t1_y >= 0 && t1_y <= 1)
		{
			const float u = 1 - t1_y;
			const float tt = t1_y * t1_y;
			const float tt3 = tt * t1_y;
			const float uu = u * u;
			const float uuu = uu * u;
			const float ry = uuu * p1_y + 3 * uu * t1_y * cp1_y + 3 * u * tt * cp2_y + tt3 * p2_y;
			
			if(ry < y1) y1 = ry;
			if(ry > y2) y2 = ry;
		}
		
		if(t2_y >= 0 && t2_y <= 1)
		{
			const float u = 1 - t2_y;
			const float tt = t2_y * t2_y;
			const float tt3 = tt * t2_y;
			const float uu = u * u;
			const float uuu = uu * u;
			const float ry = uuu * p1_y + 3 * uu * t2_y * cp1_y + 3 * u * tt * cp2_y + tt3 * p2_y;
			
			if(ry < y1) y1 = ry;
			if(ry > y2) y2 = ry;
		}
	}
	
}
