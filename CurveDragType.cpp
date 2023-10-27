enum CurveDragType
{
	
	/** Directly move the control points relative to the mouse. */
	Direct,
	
	/** Slide the control points along their axes. */
	Slide,
	
	/** Adjusts control points so that the dragged point always lines up with the mouse.
	  * Can be unpredictable when dragging near the edges of a segment. */
	Advanced,
	
}
