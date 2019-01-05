package org.team3128.common.listener.controltypes;

/**
 * Object to represent an axis of a joystick.  It has a floating-point value.
 * It implements hashCode() and equals(), so it can be used as a HashMap key.
 * @author Jamie
 *
 */
public class Axis extends Control
{
	public Axis(int index)
	{
		super(23, index);
	}
}
