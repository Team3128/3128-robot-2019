package org.team3128.common.autonomous.movement;

import org.team3128.common.autonomous.AutoUtils;
import org.team3128.common.drive.TankDrive;
import org.team3128.common.hardware.ultrasonic.IUltrasonic;
import org.team3128.common.util.PIDCalculator;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.datatypes.PIDConstants;

import edu.wpi.first.wpilibj.command.Command;

/*        _
 *       / \ 
 *      / _ \
 *     / [ ] \
 *    /  [_]  \
 *   /    _    \
 *  /    (_)    \
 * /_____________\
 * -----------------------------------------------------
 * UNTESTED CODE!
 * This class has never been tried on an actual robot.
 * It may be non or partially functional.
 * Do not make any assumptions as to its behavior!
 * And don't blink.  Not even for a second.
 * -----------------------------------------------------*/

/**
 * Command to move forward or backward to a certain ultrasonic distance.
 */
public class CmdMoveUltrasonic extends Command {

	double _cm;
	
	int _msec;
		
	long startTime;
	
	static final double OUTPUT_POWER_LIMIT = .4; //maximum allowed output power
	
	IUltrasonic ultrasonic;
	
	TankDrive drivetrain;
	
	PIDCalculator pidCalc;
	
	/**
	 * @param cm how far on the ultrasonic to move.
	 * @param threshold acceptible threshold from desired distance in cm
	 * @param msec How long the move should take. If set to 0, do not time the move
	 */
    public CmdMoveUltrasonic(IUltrasonic ultrasonic, TankDrive drivetrain, double cm, double threshold, PIDConstants pidConstants, int msec)
    {
    	_cm = cm;
    	
    	if(cm < 0)
    	{
    		throw new IllegalArgumentException("Distance cannot be negative!");
    	}
    	
    	_msec = msec;
    	    	
    	this.ultrasonic = ultrasonic;
    	
    	this.drivetrain = drivetrain;
    	
    	pidCalc = new PIDCalculator(pidConstants, 10, threshold);
    	pidCalc.setTarget(cm);
    }

    protected void initialize()
    {
		drivetrain.clearEncoders();
		startTime = System.currentTimeMillis();
    	ultrasonic.setAutoPing(true);

		
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute()
    {
		if(_msec != 0 && System.currentTimeMillis() - startTime >_msec)
		{
			drivetrain.stopMovement();
			AutoUtils.killRobot("Ultrasonic Move Overtime");
		}
		
        double output = pidCalc.update(ultrasonic.getDistance());

        output = RobotMath.clamp(output, -OUTPUT_POWER_LIMIT, OUTPUT_POWER_LIMIT);
  		drivetrain.tankDrive(output, output);
		
		//sensor reads every 500ms
		try
		{
			Thread.sleep(495);
		} 
		catch (InterruptedException e) 
		{
			e.printStackTrace();
		}
		
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished()
    {
    	return pidCalc.getNumUpdatesInsideThreshold() >= 2;
    }

    // Called once after isFinished returns true
    protected void end()
    {
		drivetrain.stopMovement();
		ultrasonic.setAutoPing(true);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted()
    {
    	
    }
}
