package org.team3128.common.listener.controltypes;

/**
 * Object to represent a button of a joystick.
 * It implements hashCode() and equals(), so it can be used as a HashMap key. 
 * @author Jamie
 *
 */
public class Button extends Control
{
	
	public Button(int index)
	{
		super(19, index);
	}
}
