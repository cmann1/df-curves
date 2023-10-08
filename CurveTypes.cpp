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

enum CurveVertexType
{
	
	/** This vertex has not been calculated/assigned yet. */
	None,
	
	/** Comes to a sharp point with no handles/control points.
	  * Only applicable for cubic and quadratic bezier curves. */
	Square,
	
	/** Control points on either side of a vertex can be moved individually.
	  * Only applicable for cubic and quadratic bezier curves. */
	Manual,
	
	/** Angles for control point on either side of a vertex are mirrored but the lengths are independent.
	  * Only applicable for cubic and quadratic bezier curves. */
	Smooth,
	
	/** Both the angles and length for control point on either side of a vertex are mirrored.
	  * Only applicable for cubic and quadratic bezier curves. */
	Mirror,
	
}

/** Controls what the results the `BaseCurve.eval` method wil return. */
enum EvalReturnType
{
	
	/** The `eval` method will calculate and return both the point and normal at `t`. */
	Both,
	
	/** The `eval` method will calculate only the point and normal at `t`. */
	Point,
	
	/** The `eval` method will calculate only the normal and normal at `t`. */
	Normal,
	
}
