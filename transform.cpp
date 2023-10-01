// TODO: Conver this into a SimpleTransform class
	
	// Global to local matrix
	protected float g2l_m00 = 0; // x axis x
	protected float g2l_m01 = 1; // x axis y
	protected float g2l_m11 = 1; // y axis x
	protected float g2l_m10 = 0; // y axis y
	
	// Local to global matrix
	protected float l2g_m00 = 0; // x axis x
	protected float l2g_m01 = 1; // x axis y
	protected float l2g_m11 = 1; // y axis x
	protected float l2g_m10 = 0; // y axis y
	
	private void update_matrices()
	{
		const float cr = cos(-rotation * DEG2RAD);
		const float sr = sin(-rotation * DEG2RAD);
		
		l2g_m00 = cr * _scale_x;
		l2g_m01 = sr * _scale_y;
		l2g_m10 = -sr * _scale_x;
		l2g_m11 = cr * _scale_y;
		
		const float dt = l2g_m00 * l2g_m11 - l2g_m01 * l2g_m10;
		if(dt != 0)
		{
			g2l_m00 = l2g_m11 / dt;
			g2l_m01 = -l2g_m01 / dt;
			g2l_m10 = -l2g_m10 / dt;
			g2l_m11 = l2g_m00 / dt;
		}
		else
		{
			g2l_m00 = g2l_m01 = g2l_m10 = g2l_m11 = 0;
		}
	}
	
	/// Converts from the global coordinate system to this curve's local one.
	void global_to_local(const float x, const float y, float &out out_x, float &out out_y)
	{
		out_x = (x - this.x) * g2l_m00 + (y - this.y) * g2l_m01;
		out_y = (x - this.x) * g2l_m10 + (y - this.y) * g2l_m11;
	}
	
	/// Converts from this curve's local coordinate system to the global one.
	void local_to_global(const float x, const float y, float &out out_x, float &out out_y)
	{
		out_x = x * l2g_m00 + y * l2g_m01 + this.x;
		out_y = x * l2g_m10 + y * l2g_m11 + this.y;
	}
	
