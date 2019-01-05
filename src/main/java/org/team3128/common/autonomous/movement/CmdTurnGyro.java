package org.team3128.common.autonomous.movement;

import org.team3128.common.drive.ITankDrive;
import org.team3128.common.util.PIDCalculator;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.datatypes.PIDConstants;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Gyro;

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
public class CmdTurnGyro extends Command {

	double degrees;
		
	double threshold;
		
	static final double OUTPUT_POWER_LIMIT = .5; //maximum allowed output power
	
	private Gyro gyro;
	
	private ITankDrive drivetrain;
	
	private PIDCalculator pidCalc;
	
	
    public CmdTurnGyro(Gyro gyro, ITankDrive drivetrain, double degrees, double threshold, PIDConstants pidConstants, int msec)
    {
    	super(msec / 1000.0);
    	this.degrees = degrees;
    	
    	this.threshold = threshold;
    	this.gyro = gyro;
    	this.pidCalc = new PIDCalculator(pidConstants, 10, threshold);
    	this.drivetrain = drivetrain;
    	
    	pidCalc.setTarget(degrees);
    	pidCalc.enableLogging("Gyro Turn");
    }

    protected void initialize()
    {
		gyro.reset();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute()
    {
		
		double output = pidCalc.update(gyro.getAngle());
        		
        output = RobotMath.clamp(output, -OUTPUT_POWER_LIMIT, OUTPUT_POWER_LIMIT);
   		drivetrain.tankDrive(-output, output);
		
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished()
    {
    	return isTimedOut() || pidCalc.getNumUpdatesInsideThreshold() >= 10;
    }

    // Called once after isFinished returns true
    protected void end()
    {
		drivetrain.stopMovement();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted()
    {
    	
    }
}
