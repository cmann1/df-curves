class Point
{
	
	[persist] float x;
	[persist] float y;
	
	Point() { }
	
	Point(const float x, const float y)
	{
		this.x = x;
		this.y = y;
	}
	
}

/// A point with a weight.
class PointW : Point
{
	
	float w;
	
}
