enum CurveType
{
	
	/** Each vertex is connected with a straight line. */
	Linear,
	
	/** One control point per vertex, but smoothed vertices affect the entire spline. */
	QuadraticBezier,
	
	/** Two control points per vertex, giving but more local control. */
	CubicBezier,
	
	/** Passes through all vertices smoothly without any control points.
	  * Tension can be controlled. */
	CatmullRom,
	
	/** Gives a smooth curve without control points but the spline does not pass through vertices.
	  * Segment shape can be controlled per vertex with a weight factor. */
	BSpline,
	
}

/** Controls how the beginning and end control points are calculated for CatmullRom splines. */
enum CurveEndControl
{
	
	/** End control points are calculated automatically by extrapolating the two end vertices. */
	Automatic,
	
	/** Same as Automatic, but if the path has 3 or more vertices also takes the angle of the third vertex into account. */
	AutomaticAngle,
	
	/** Allows manual control/placement of the control points . */
	Manual,
	
}
