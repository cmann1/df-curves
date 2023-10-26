#include '../lib/math/math.cpp';

/** Handles logic for dragging, mirroring, and resetting curve control points. */
class CurveControlPointDrag
{
	
	bool drag_vertex;
	
	CurveControlPoint@ point;
	CurveVertex@ vertex;
	float x, y;
	float start_x, start_y;
	float offset_x, offset_y;
	float angle;
	float length;
	int vertex_index;
	int segment_index;
	CurveControlType point_type;
	CurveControlType vertex_type;
	
	CurveControlPoint@ mirror_point;
	float mirror_dx, mirror_dy;
	float mirror_start_x, mirror_start_y;
	int mirror_vertex_index;
	float mirror_length;
	float mirror_length_ratio;
	CurveControlType mirror_point_type;
	
	CurveControlPoint@ axis;
	
	bool start_drag(
		MultiCurve@ curve,
		CurveControlPoint@ point, const float x, const float y, const int dir=-1)
	{
		if(@this.vertex != null || @this.point != null)
			return false;
		if(@point == null || @point.vertex == null)
			return false;
		if(curve.type != QuadraticBezier && curve.type != CubicBezier)
			return false;
		if(curve.type == QuadraticBezier && @point != @point.vertex.quad_control_point)
			return false;
		if(curve.type == CubicBezier && @point != @point.vertex.cubic_control_point_1 && @point != @point.vertex.cubic_control_point_2)
			return false;
		
		vertex_index = curve.vertices.findByRef(point.vertex);
		if(vertex_index == -1)
			return false;
		
		drag_vertex = false;
		
		segment_index = vertex_index;
		@this.point = point;
		@vertex = point.vertex;
		this.x = x;
		this.y = y;
		start_x = point.x;
		start_y = point.y;
		offset_x = point.x - x;
		offset_y = point.y - y;
		point_type = point.type;
		vertex_type = vertex.type;
		
		length = sqrt(point.x * point.x + point.y * point.y);
		
		if(curve.type == QuadraticBezier)
		{
			mirror_vertex_index = mod(segment_index + dir, curve.vertex_count);
			@mirror_point = @curve.vertices[mirror_vertex_index].quad_control_point;
			float dx, dy;
			mirror_delta(dx, dy, mirror_point);
			angle = angle_between(dx, dy, point.x, point.y);
			mirror_length = magnitude(dx, dy);
			
			if(dir > 0)
			{
				@vertex = curve.vertices[mirror_vertex_index];
				mirror_delta(dx, dy, mirror_point);
				mirror_length = magnitude(dx, dy);
			}
			
			if(curve.closed || vertex_index < curve.vertex_count - 1)
			{
				@axis = @curve.vertices[mod(vertex_index - dir, curve.vertex_count)].quad_control_point;
			}
		}
		else if(curve.type == CubicBezier)
		{
			@mirror_point = @point == @point.vertex.cubic_control_point_1
				? @point.vertex.cubic_control_point_2 : @point.vertex.cubic_control_point_1;
			segment_index = mod(@point == @point.vertex.cubic_control_point_1 ? vertex_index - 1 : vertex_index, curve.vertex_count);
			mirror_vertex_index = mod(segment_index + (@point == @point.vertex.cubic_control_point_1 ? 1 : -1), curve.vertex_count);
			angle = angle_between(mirror_point.x, mirror_point.y, point.x, point.y);
			mirror_length = magnitude(mirror_point.x, mirror_point.y);
		}
		
		if(@mirror_point != null)
		{
			mirror_start_x = mirror_point.x;
			mirror_start_y = mirror_point.y;
			mirror_length_ratio = length != 0 ? mirror_length / length : 0;
			
			mirror_point_type = mirror_point.type;
		}
		
		return true;
	}
	
	bool do_drag(
		MultiCurve@ curve, const float x, const float y,
		const ControlPointMirrorType mirror, const bool constrain_to_axis, const bool update_point=true)
	{
		if(@point == null)
			return false;
		if(x == this.x && y == this.y)
			return false;
		
		this.x = x;
		this.y = y;
		
		if(update_point)
		{
			point.x = x + offset_x;
			point.y = y + offset_y;
		}
		
		float dx, dy;
		mirror_delta(dx, dy);
		length = magnitude(dx, dy);
		
		if(curve.type == QuadraticBezier && @axis != null && constrain_to_axis && update_point)
		{
			const float length = magnitude(axis.x, axis.y);
			if(length != 0)
			{
				const float nx = axis.x / length;
				const float ny = axis.y / length;
				const float ax = point.x + point.vertex.x - axis.vertex.x;
				const float ay = point.y + point.vertex.y - axis.vertex.y;
				const float dp = dot(ax, ay, nx, ny);
				point.x = axis.vertex.x + dp * nx - point.vertex.x;
				point.y = axis.vertex.y + dp * ny - point.vertex.y;
			}
		}
		
		const bool maintain_angle = mirror == MaintainAngle && (point.type != Smooth || mirror_point.type != Smooth);
		
		if(mirror != MaintainAngle && @mirror_point != null)
		{
			float dx2, dy2;
			mirror_delta(dx, dy);
			mirror_delta(dx2, dy2, mirror_point);
			angle = angle_between(dx2, dy2, dx, dy);
		}
		
		const bool mirror_quad = curve.type == QuadraticBezier && @mirror_point != null && (mirror_point.type == Smooth || maintain_angle);
		
		if(
			mirror_quad ||
			// Cubic
			curve.type == CubicBezier && @mirror_point != null &&
			(point.type == Smooth && mirror_point.type == Smooth || maintain_angle))
		{
			float new_length = mirror_length;
			switch(mirror)
			{
				case Angle:
					new_length = mirror_length;
					break;
				case Length:
					mirror_delta(dx, dy);
					new_length = magnitude(dx, dy);
					mirror_length = new_length;
					break;
				case LengthRatio:
					mirror_delta(dx, dy);
					new_length = magnitude(dx, dy) * mirror_length_ratio;
					mirror_length = new_length;
					break;
			}
			
			mirror_delta(dx, dy);
			const float new_angle = atan2(dy, dx) - (maintain_angle ? angle : PI);
			mirror_point.x = cos(new_angle) * new_length;
			mirror_point.y = sin(new_angle) * new_length;
			
			if(mirror_quad)
			{
				mirror_point.x += vertex.x - mirror_point.vertex.x;
				mirror_point.y += vertex.y - mirror_point.vertex.y;
				
				if(mirror != LengthRatio)
				{
					mirror_delta(dx, dy, mirror_point);
					new_length = magnitude(dx, dy);
				}
			}
			else if(mirror != LengthRatio)
			{
				mirror_delta(dx, dy, mirror_point);
				new_length = magnitude(dx, dy);
			}
			
			if(mirror != LengthRatio)
			{
				mirror_length_ratio = length != 0 ? new_length / length : 0;
			}
		}
		
		if(mirror_vertex_index != -1 && mirror_vertex_index != segment_index)
		{
			curve.invalidate(mirror_vertex_index);
		}
		
		if(curve.closed || segment_index < curve.vertex_count - 1)
		{
			curve.invalidate(segment_index, true);
		}
		
		return true;
	}
	
	bool stop_drag(MultiCurve@ curve, const bool accept=true)
	{
		if(@point == null)
			return false;
		
		if(!accept)
		{
			cancel_drag(curve);
		}
		
		clear_drag();
		
		return true;
	}
	
	bool start_drag_vertex(
		MultiCurve@ curve,
		CurveVertex@ vertex, const float x, const float y)
	{
		if(@this.vertex != null || @this.point != null)
			return false;
		if(@vertex == null)
			return false;
		
		vertex_index = curve.vertices.findByRef(vertex);
		if(vertex_index == -1)
			return false;
		
		drag_vertex = true;
		
		segment_index = vertex_index;
		@this.vertex = vertex;
		this.x = x;
		this.y = y;
		start_x = vertex.x;
		start_y = vertex.y;
		offset_x = vertex.x - x;
		offset_y = vertex.y - y;
		vertex_type = vertex.type;
		
		if(curve.type == QuadraticBezier && (curve.closed || vertex_index > 0 && vertex_index < curve.vertex_count - 1))
		{
			mirror_vertex_index = mod(segment_index - 1, curve.vertex_count);
			@mirror_point = @curve.vertices[mirror_vertex_index].quad_control_point;
			mirror_start_x = mirror_point.x;
			mirror_start_y = mirror_point.y;
			mirror_dx = mirror_point.x + mirror_point.vertex.x - vertex.x;
			mirror_dy = mirror_point.y + mirror_point.vertex.y - vertex.y;
			
			mirror_point_type = mirror_point.type;
		}
		
		return true;
	}
	
	bool do_drag_vertex(MultiCurve@ curve, const float x, const float y)
	{
		if(@vertex == null)
			return false;
		if(x == this.x && y == this.y)
			return false;
		
		this.x = x;
		this.y = y;
		vertex.x = x + offset_x;
		vertex.y = y + offset_y;
		
		if(curve.type == QuadraticBezier && @mirror_point != null && mirror_point.type == Smooth)
		{
			mirror_point.x = vertex.x + mirror_dx - mirror_point.vertex.x;
			mirror_point.y = vertex.y + mirror_dy - mirror_point.vertex.y;
			
			if(mirror_vertex_index != segment_index)
			{
				curve.invalidate(mirror_vertex_index);
			}
		}
		
		curve.invalidate(vertex_index);
		
		return true;
	}
	
	bool stop_drag_vertex(MultiCurve@ curve, const bool accept=true)
	{
		if(@vertex == null)
			return false;
		
		if(!accept)
		{
			cancel_drag(curve);
		}
		
		clear_drag();
		
		return true;
	}
	
	private void cancel_drag(MultiCurve@ curve)
	{
		if(@point != null)
		{
			point.x = start_x;
			point.y = start_y;
			point.type = point_type;
		}
		if(drag_vertex && @vertex != null)
		{
			vertex.x = start_x;
			vertex.y = start_y;
			vertex.type = vertex_type;
		}
		
		if(@mirror_point != null)
		{
			mirror_point.x = mirror_start_x;
			mirror_point.y = mirror_start_y;
			
			mirror_point.type = mirror_point_type;
			
			if(mirror_vertex_index != segment_index)
			{
				curve.invalidate(mirror_vertex_index, true);
			}
		}
		
		curve.invalidate(vertex_index, !drag_vertex);
	}
	
	private void clear_drag()
	{
		@point = null;
		@vertex = null;
		@mirror_point = null;
		@axis = null;
		vertex_index = -1;
		mirror_vertex_index = -1;
	}
	
	private void mirror_delta(float &out dx, float &out dy, CurveControlPoint@ point=null)
	{
		@point = @point == null ? this.point : point;
		
		if(@point.vertex == @vertex)
		{
			dx = point.x;
			dy = point.y;
		}
		else
		{
			dx = point.x + point.vertex.x - vertex.x;
			dy = point.y + point.vertex.y - vertex.y;
		}
	}
	
}
