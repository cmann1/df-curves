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
	
	string get_vertex_type_name(const CurveVertexType type)
	{
		switch(type)
		{
			case CurveVertexType::Square: return 'Square';
			case CurveVertexType::Manual: return 'Manual';
			case CurveVertexType::Smooth: return 'Smooth';
			case CurveVertexType::Mirror: return 'Mirror';
			case CurveVertexType::None:
			default:
				return 'None';
		}
		
		return 'Unknown';
	}
	
}
