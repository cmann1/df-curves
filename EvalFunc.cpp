namespace Curve
{
	
	/** A function to evaluate a curve at the given segment index and t value.
	  * @param segment The segment index.
	  * @param t The t value within the segment in the range 0..1.
	  * @param x The x value of the returned point on the curve.
	  * @param y The y value of the returned point on the curve.
	  * @param normal_x The x value of the normal of the returned point on the curve.
	  * @param normal_y The y value of the normal of the returned point on the curve.
	  * @param normalise Set to false to skip normalisation of normal vectors. */
	funcdef void EvalFunc(const int, const float, float &out, float &out, float &out, float &out, const bool);
	
}
