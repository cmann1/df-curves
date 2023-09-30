class CurveVertex : Point
{
	
	[option,1:Square,Manual,Smooth,Mirror] CurveVertexType type = Smooth;
	/// The weight for either conic quadratic beziers or b-splines for the segment starting at this vertex.
	[persist] float weight = 1;
	/// A per-segment tension for CatmullRom splines/
	[persist] float tension = 1;
	
	/// Has the segment starting with this segment been invalidated/changed, meaning that the arc length
	/// and segments look up table need to be recalculated.
	bool invalidated = true;
	/// The approximated length of curve the segment starting with this vertex.
	float length;
	/// A precomputed set of points along the curve, also mapping raw t values to real distances/uniform t values along the curve.
	array<CurveSegment> segments;
	
	CurveVertex()
	{
	}
	
	CurveVertex(const float x, const float y)
	{
		this.x = x;
		this.y = y;
	}
	
	CurveVertex@ extrapolate(
		const CurveVertex@ p1, const CurveVertex@ p2, const CurveVertex@ p3=null,
		const float angle_multiplier=1, const float length_multiplier=1)
	{
		// Simple just extend the two points.
		if(@p3 == null)
		{
			x = p1.x + (p1.x - p2.x);
			y = p1.y + (p1.y - p2.y);
		}
		/// With a third point also take the angle difference between the 3 points into account to rotate the extended point.
		else
		{
			const float a1 = atan2(p1.y - p2.y, p1.x - p2.x);
			const float a2 = shortest_angle(atan2(p2.y - p3.y, p2.x - p3.x), atan2(p1.y - p3.y, p1.x - p3.x));
			const float a = a1 + a2 * angle_multiplier;
			const float length = sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y)) * length_multiplier;
			x = p1.x + cos(a) * length;
			y = p1.y + sin(a) * length;
		}
		
		return this;
	}
	
	CurveVertex@ copy_from(const CurveVertex@ p)
	{
		x = p.x;
		y = p.y;
		return this;
	}
	
}

class CurveSegment
{
	
	/// The real/raw t value of this point relative to its segment.
	float t_real;
	/// The uniform t value based on the approximated distance of this point from the start of the segment.
	float t_uniform;
	
}
