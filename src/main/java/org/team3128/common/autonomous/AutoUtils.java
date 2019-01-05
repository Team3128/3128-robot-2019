package org.team3128.common.autonomous;

import org.team3128.common.util.Log;

import edu.wpi.first.wpilibj.command.Scheduler;

/**
 * Various functions used in the autonomous code.
 * @author Jamie
 *
 */
public class AutoUtils
{


	/**
	 * 
	 * Stops the autonomous program while still allowing the teleop to run normally once teleop starts
	 * 
	 * Things that call this should stop any motors that were running first!
	 * @param cause
	 */
	public static void killRobot(String cause)
	{
		Log.fatal("AutoUtils", "Robot stopped by autonomous error: " + cause);
		
		//stop more commands from being run
		Scheduler.getInstance().disable();
	
		
		//BAD, KILLS ROBOT
		//throw new RuntimeException("Error in automatic movement - " + cause + "\nRobot shut down!");
	}
}
