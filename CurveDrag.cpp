#include 'cubic_projection_identity.cpp';
#include 'cubic_projection_identity_rational.cpp';
#include 'quadratic_projection_identity.cpp';
#include 'quadratic_projection_identity_rational.cpp';

/** Handles logic for dragging/molding, and resetting a curve segment. */
class CurveDrag
{
	
	bool busy;
	
	float x, y;
	float offset_x, offset_y;
	int segment;
	float t;
	float u;
	float ratio;
	
	float b_r;
	
	bool start(MultiCurve@ curve, const int segment, const float t)
	{
		if(curve.type != QuadraticBezier && curve.type != CubicBezier)
			return false;
		if(t < 0 || t > 1)
			return false;
		if(segment < 0 || segment > curve.vertex_count - (curve.closed ? 1 : 2))
			return false;
		
		busy = true;
		
		this.segment = segment;
		this.t = t;
		
		if(curve.type == QuadraticBezier)
			QuadraticBezier::calc_abc_ratio(t, u, ratio);
		else
			CubicBezier::calc_abc_ratio(t, u, ratio);
		
		b_r = curve.eval_ratio(segment, t);
		
		if(curve.type == CubicBezier)
		{
			
		}
		
		return true;
	}
	
}
