package org.team3128.common.listener.controllers;

import org.team3128.common.listener.controltypes.Axis;



/**
 * Controller object for a Logitech Attack joystick.
 * 
 * @author Jamie
 */
public class ControllerAttackJoy
{
		public static final Axis JOYX = new Axis(0);
		public static final Axis JOYY = new Axis(1);
		public static final Axis THROTTLE = new Axis(2);

		
		private ControllerAttackJoy()
		{
			
		}
		
		public static final ControllerAttackJoy instance = new ControllerAttackJoy();
}
