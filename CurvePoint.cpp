class CurvePoint
{
	
	[persist] float x;
	[persist] float y;
	
	CurvePoint() { }
	
	CurvePoint(const float x, const float y)
	{
		this.x = x;
		this.y = y;
	}
	
	void set(const float x, const float y)
	{
		this.x = x;
		this.y = y;
	}
	
}

/// A point with a weight.
class CurvePointW : CurvePoint
{
	
	float w;
	
}
