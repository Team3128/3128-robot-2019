package org.team3128.common.listener.controllers;

import org.team3128.common.listener.controltypes.Axis;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.listener.controltypes.POV;

/**
 * Controller object for a Logitech Extreme3D joystick.
 * 
 * NOTE: I've omitted all of the numbered buttons. You can just use the new Button constructor.
 * @author Jamie
 */
public class ControllerExtreme3D
{
		public static final Button TRIGGER = new Button(1);
		
		public static final Axis JOYX = new Axis(0);
		public static final Axis JOYY = new Axis(1);
		public static final Axis TWIST = new Axis(2);
		public static final Axis THROTTLE = new Axis(3);

		public static final POV POV = new POV(0);
		private ControllerExtreme3D()
		{
			
		}
}
