package org.team3128.common.listener.controltypes;


/**
 * Control type for the POV stick/DPAD, which is read as an angle by WPILib.
 *
 * @author Narwhal
 *
 */
public class POV extends Control
{
	public POV(int index)
	{
		super(7, index);
	}

}
