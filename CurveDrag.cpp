#include 'cubic_projection_identity.cpp';
#include 'cubic_projection_identity_rational.cpp';
#include 'quadratic_projection_identity.cpp';
#include 'quadratic_projection_identity_rational.cpp';

/** Handles logic for dragging/molding a curve segment. */
class CurveDrag
{
	
	bool busy;
	bool is_linear;
	
	MultiCurve@ curve;
	CurveType type;
	float x, y;
	float offset_x, offset_y;
	int segment;
	float t;
	CurveVertex@ p1, p2;
	CurveControlPoint@ cp1, cp2;
	bool is_rational;
	float u;
	float ratio;
	
	float br;
	float e1x, e1y, e2x, e2y;
	float e1r, e2r;
	
	bool start(MultiCurve@ curve, const int segment, const float t, const float x, const float y)
	{
		if(busy)
			return false;
		if(curve.type != QuadraticBezier && curve.type != CubicBezier)
			return false;
		if(t < 0 || t > 1)
			return false;
		if(segment < 0 || segment > curve.vertex_count - (curve.closed ? 1 : 2))
			return false;
		
		busy = true;
		this.x = x;
		this.y = y;
		
		@this.curve = curve;
		type = curve.type;
		this.segment = segment;
		this.t = t;
		@p1 = curve.vert(segment);
		@p2 = curve.vert(segment + 1);
		
		if(type == CubicBezier)
		{
			@cp1 = @p1.cubic_control_point_2;
			@cp2 = @p2.cubic_control_point_1;
			if(cp1.type == Square || cp2.type == Square)
			{
				@cp1 = cp1.type == Square ? cp2 : cp1;
				type = QuadraticBezier;
			}
		}
		
		if(type == QuadraticBezier)
			QuadraticBezier::calc_abc_ratio(t, u, ratio);
		else
			CubicBezier::calc_abc_ratio(t, u, ratio);
		
		br = curve.eval_ratio(segment, t);
		
		if(type == QuadraticBezier)
		{
			if(curve.type == QuadraticBezier)
			{
				@cp1 = @p1.quad_control_point;
			}
			is_rational = p1.weight != cp1.weight || cp1.weight != p2.weight;
		}
		else
		{
			is_rational = p1.weight != cp1.weight || cp1.weight != cp2.weight || cp2.weight != p2.weight;
			
			if(!is_rational)
			{
				CubicBezier::calc_abc_tangent(
					p1.x, p1.y,
					p1.x + cp1.x, p1.y + cp1.y,
					p2.x + cp2.x, p2.y + cp2.y,
					p2.x, p2.y,
					t, u, ratio, x, y,
					e1x, e1y, e2x, e2y);
			}
			else
			{
				CubicBezier::calc_abc_tangent(
					p1.x, p1.y, p1.weight,
					p1.x + cp1.x, p1.y + cp1.y, cp1.weight,
					p2.x + cp2.x, p2.y + cp2.y, cp2.weight,
					p2.x, p2.y, p2.weight,
					t, u, ratio, x, y, br,
					e1x, e1y, e1r, e2x, e2y, e2r);
			}
		}
		
		return true;
	}
	
	bool update(const float x, const float y)
	{
		if(!busy)
			return false;
		if(x == this.x && y == this.y)
			return false;
		
		this.x = x;
		this.y = y;
		
		if(type == QuadraticBezier)
		{
			float ax, ay, ar, cx, cy, cr;
			
			if(!is_rational)
			{
				QuadraticBezier::calc_abc(
					p1.x, p1.y, p2.x, p2.y,
					u, ratio, x, y,
					ax, ay, cx, cy);
			}
			else
			{
				QuadraticBezier::calc_abc(
					p1.x, p1.y, p1.weight, p2.x, p2.y, p2.weight,
					u, ratio, x, y, br,
					ax, ay, ar, cx, cy, cr);
			}
			
			cp1.x = ax - cp1.vertex.x;
			cp1.y = ay - cp1.vertex.y;
		}
		else
		{
			float cp1x, cp1y, cp1r;
			float cp2x, cp2y, cp2r;
			
			if(!is_rational)
			{
				CubicBezier::calc_from_abc_tangent(
					p1.x, p1.y, p2.x, p2.y,
					e1x, e1y, e2x, e2y,
					t, u, ratio, x, y,
					cp1x, cp1y, cp2x, cp2y);
			}
			else
			{
				CubicBezier::calc_from_abc_tangent(
					p1.x, p1.y, p1.weight, p2.x, p2.y, p2.weight,
					e1x, e1y, e1r, e2x, e2y, e2r,
					t, u, ratio, x, y, br,
					cp1x, cp1y, cp1r, cp2x, cp2y, cp2r);
			}
			
			cp1.x = cp1x - p1.x;
			cp1.y = cp1y - p1.y;
			cp2.x = cp2x - p2.x;
			cp2.y = cp2y - p2.y;
		}
		
		return true;
	}
	
	bool stop()
	{
		if(!busy)
			return false;
		
		busy = false;
		@p1 = null;
		@p2 = null;
		@cp1 = null;
		@cp2 = null;
		
		return true;
	}
	
}
