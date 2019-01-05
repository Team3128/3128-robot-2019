package org.team3128.common.listener.controllers;

import org.team3128.common.listener.controltypes.Axis;
import org.team3128.common.listener.controltypes.Button;


/**
 * Controller object for an Xbox controller.
 * @author Jamie
 */
public class ControllerXbox
{
	public static final Button A = new Button(1);
	public static final Button B = new Button(2);
	public static final Button X = new Button(3);
	public static final Button Y = new Button(4);
	public static final Button LB = new Button(5);
	public static final Button RB = new Button(6);
	public static final Button BACK = new Button(7);
	public static final Button START = new Button(8);
	public static final Button L3 = new Button(9);
	public static final Button R3 = new Button(10);

	public static final Axis JOY1X = new Axis(0);
	public static final Axis JOY1Y = new Axis(1);
	public static final Axis TRIGGERL = new Axis(2);
	public static final Axis TRIGGERR = new Axis(3);
	public static final Axis JOY2X = new Axis(4);
	public static final Axis JOY2Y = new Axis(5);

		
	private ControllerXbox()
	{
		
	}
		
}
