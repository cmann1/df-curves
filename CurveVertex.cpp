#include 'CurvePoint.cpp';
#include 'CurveTypes.cpp';

class CurveControlPoint : CurvePoint
{
	
	[persist] float weight = 1;
	
	CurveControlPoint() { }
	
	CurveControlPoint(const float x, const float y)
	{
		this.x = x;
		this.y = y;
	}
	
	CurveControlPoint@ added(const CurveControlPoint@ p1, const CurveControlPoint@ p2)
	{
		x = p1.x + p2.x;
		y = p1.y + p2.y;
		this.weight = p1.weight;
		return this;
	}
	
}

class CurveVertex : CurveControlPoint
{
	
	[option,1:Square,Manual,Smooth,Mirror]
	CurveVertexType type = Smooth;
	/** A per-segment tension for CatmullRom splines/ */
	[persist] float tension = 1;
	
	/** The right hand side control point for this vertex. Only applicable to quadratic bezier curves. */
	[persist] CurveControlPoint quad_control_point(NAN, NAN);
	
	/** The left hand side control point for this vertex. Only applicable to cubic bezier curves. */
	[persist] CurveControlPoint cubic_control_point_1(NAN, NAN);
	/** The right hand side control point for this vertex. Only applicable to cubic bezier curves. */
	[persist] CurveControlPoint cubic_control_point_2(NAN, NAN);
	
	/** The bounding box of this curve segment. */
	float x1, y1;
	float x2, y2;
	
	/** The approximated length of the curve segment starting with this vertex. */
	float length;
	
	/** A precomputed set of points along the curve, mapping raw t values to real distances/uniform t values along the curve. */
	array<CurveArc> arcs;
	int arc_count;
	
	CurveVertex() { }
	
	CurveVertex(const float x, const float y)
	{
		this.x = x;
		this.y = y;
	}
	
	CurveVertex@ extrapolate(
		const CurveVertex@ p1, const CurveVertex@ p2, const CurveVertex@ p3=null,
		const float angle_multiplier=0.75, const float length_multiplier=1)
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
			const float a2 = clamp(shortest_angle(a1, atan2(p2.y - p3.y, p2.x - p3.x)), -90 * DEG2RAD, 90 * DEG2RAD);
			const float a = a1 - a2 * angle_multiplier;
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
		weight = p.weight;
		return this;
	}
	
	CurveArc@ arc_from_end(const int offset=0)
	{
		return arc_count > offset ? arcs[arc_count - 1 - offset] : arcs[0];
	}
	
	CurveArc@ arc_from_start(const int offset=0)
	{
		return offset < arc_count ? arcs[offset] : arcs[0];
	}
	
}

class CurveArc
{
	
	float x;
	float y;
	/** The total length from the start of the curve to the end of this arc. */
	float length;
	/** The t value of this point relative to its segment. */
	float t;
	
}
