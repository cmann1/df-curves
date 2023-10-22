/** See `Curve::calculate_arc_lengths` for descriptions of these properties. */
class MultiCuveSubdivisionSettings
{
	
	/** `division_count` */
	int count = 6;
	
	/** Specified in degrees. */
	float angle_min = 5;
	
	float max_stretch_factor = 0.2;
	
	float length_min = 0;
	
	int max_subdivisions = 4;
	
	/** Specified in degrees. */
	float angle_max = 65;
	
	float length_max = 0;
	
}
