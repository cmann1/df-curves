namespace Curve
{
	
	string get_type_name(const CurveType type)
	{
		switch(type)
		{
			case CurveType::Linear: return 'Linear';
			case CurveType::QuadraticBezier: return 'QuadraticBezier';
			case CurveType::CubicBezier: return 'CubicBezier';
			case CurveType::CatmullRom: return 'CatmullRom';
			case CurveType::BSpline: return 'BSpline';
		}
		
		return 'Unknown';
	}
	
	string get_control_type_name(const CurveControlType type)
	{
		switch(type)
		{
			case CurveControlType::Square: return 'Square';
			case CurveControlType::Manual: return 'Manual';
			case CurveControlType::Smooth: return 'Smooth';
			case CurveControlType::None:
			default:
				return 'None';
		}
		
		return 'Unknown';
	}
	
}
