package org.team3128.common.drive;

import static java.lang.Math.abs;

import org.team3128.common.autonomous.AutoUtils;
import org.team3128.common.hardware.encoder.both.QuadratureEncoder;
import org.team3128.common.hardware.motor.MotorGroup;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.VelocityPID;
import org.team3128.common.util.datatypes.PIDConstants;
import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Angle;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Class which represents a tank drive on a robot.
 * 
 * Also provides commands for autonomous movement.
 * @author Jamie
 *
 */
public class TankDrive implements ITankDrive
{
	private MotorGroup leftMotors;
    	
    private MotorGroup rightMotors;
    
	private QuadratureEncoder encLeft;
	private QuadratureEncoder encRight;
	
    
    /**
     * circumference of wheels in cm
     */
    public final double wheelCircumfrence;
    
    /**
     * horizontal distance between wheels in cm
     */
    public final double wheelBase;
    
    /**
     * distance between front and back wheels
     */
    public final double track;
    
    /**
     * Circumference of the turning circle when in-place turning
     */
    public final double turningCircleCircumference;
    
    /**
     * Ratio between turns of the wheels to turns of the encoder
     */
    private double gearRatio;
    
    public double getGearRatio()
	{
		return gearRatio;
	}

	public void setGearRatio(double gearRatio)
	{
		this.gearRatio = gearRatio;
	}

	/**
     * 
     * @param leftMotors The motors on the left side of the robot
     * @param rightMotors The motors on the riht side of the robot.
     * @param encLeft The encoder on the left motors
     * @param encRight The encoder on the right motors
     * @param wheelCircumfrence The circumference of the wheel
     * @param gearRatio The gear ratio of the turns of the wheels per turn of the encoder shaft
     * @param wheelBase The distance between the front and back wheel on a side
     * @param track distance between front and back wheels
     */
    public TankDrive(MotorGroup leftMotors, MotorGroup rightMotors, QuadratureEncoder encLeft, QuadratureEncoder encRight, double wheelCircumfrence, double gearRatio, double wheelBase, double track)
    {
    	this.leftMotors = leftMotors;
    	this.rightMotors = rightMotors;
    	
    	this.encLeft = encLeft;
    	this.encRight = encRight;
    	
    	this.wheelCircumfrence = wheelCircumfrence;
    	this.wheelBase = wheelBase;
    	this.track = track;
    	this.gearRatio = gearRatio;
    	
    	double turningCircleDiameter = Math.sqrt(RobotMath.square(track) + RobotMath.square(wheelBase)); //pythagorean theorem
    	turningCircleCircumference = turningCircleDiameter * Math.PI;
    	
    	if(gearRatio <= 0)
    	{
    		throw new IllegalArgumentException("Invalid gear ratio");
    	}
    }
    
	//threshold below which joystick movements are ignored.
	final static double thresh = 0.2;
	
	
	/**
	 * Update the motor outputs with the given control values.
	 * @param joyX horizontal control input
	 * @param joyY vertical control input
	 * @param throttle throttle control input scaled between 1 and -1 (-.8 is 10 %, 0 is 50%, 1.0 is 100%)
	 */
	@Override
    public void arcadeDrive(double joyX, double joyY, double throttle, boolean fullSpeed)
    {
    	
        double spdL, spdR;
    	//read joystick values
    	joyX = Math.abs(joyX) > thresh ? -1 * joyX : 0.0;
    	
       	joyY = Math.abs(joyY) > thresh ? -1 * joyY : 0.0;
    	
    	if(!fullSpeed)
    	{
    		joyY *= .65;
    	}
    	else
    	{
    		joyY *= 1;
    	}
    	
    	//scale from 1 to -1 to 1 to 0
    	throttle =  ( throttle + 1) / 2;

    	if(throttle < .3)
    	{
    		throttle = .3;
    	}
    	else if(throttle > .8)
    	{
    		throttle = 1;
    	}
    	
    	joyY *= throttle;
    	joyX *= throttle;
    	
    	spdR = RobotMath.clampPosNeg1(joyY + joyX);
    	spdL = RobotMath.clampPosNeg1(joyY - joyX);
    	
    	//Log.debug("TankDrive", "x1: " + joyX + " throttle: " + throttle + " spdR: " + spdR + " spdL: " + spdL);

    	leftMotors.setTarget(spdL);
    	rightMotors.setTarget(spdR);
    }
    
    /**
     * Drive by providing motor powers for each side.
     * @param powL the left side power.
     * @param powR the right side power.
     */
	@Override
    public void tankDrive(double powL, double powR)
    {
    	leftMotors.setTarget(powL);
    	rightMotors.setTarget(powR);
    }
    
	public void clearEncoders()
	{
		encLeft.reset();
		encRight.reset();
	}

	@Override
	public void stopMovement()
	{
		leftMotors.setTarget(0);
		rightMotors.setTarget(0);
	}
	
	/**
	 * Get the estimated angle that the robot has turned since the encoders were last reset, based on the relative distances of each side.
	 * 
	 * Range: [0, 360)
	 * 0 degrees is straight ahead.
	 * @return
	 */
	@Override
	public double getRobotAngle()
	{
		double leftDist = encDistanceToCm(encLeft.getAngle());
		double rightDist = encDistanceToCm(encRight.getAngle());
		
		double difference = leftDist - rightDist;
		
		return RobotMath.normalizeAngle((difference / turningCircleCircumference) * Angle.ROTATIONS);
	}
	
	/**
	 * Convert cm of robot movement to encoder movement in degrees
	 * @param cm
	 * @param wheelCircumference the circumference of the wheels
	 * @return
	 */
	double cmToEncDegrees(double cm)
	{
		return (cm * 360) / (wheelCircumfrence * gearRatio);
	}
	
	/**
	 * Convert cm of robot movement to encoder rotations
	 * @param cm
	 * @param wheelCircumference the circumference of the wheels
	 * @return
	 */
	double encDistanceToCm(double encDistance)
	{
		return (encDistance / 360) * wheelCircumfrence * gearRatio;
	}
	
    /**
     * Enum for how CmdMoveDistance determines when to end a move command.
     * @author Jamie
     *
     */
	public enum MoveEndMode
	{
		BOTH, //ends when both sides have reached their targets.  Keeps moving both sides until then.  Make sure both encoders are working!
		EITHER, //Stops both sides when either side has reached its target.
		PER_SIDE //Stops each side when it reaches its target.  The command ends when both sides hit their targets.
	}
    
  
	/**
     * Command to move each side of the drivetrain a specified distance.
     * 
     * Common logic shared by all of the autonomous movement commands
     */
    public class CmdMoveDistance extends Command {
    	protected double power;
    	
    	protected double leftDist, rightDist;
    	
    	protected MoveEndMode endMode;
    	
    	/**
    	 * @param leftDist Degrees to spin the left wheel
    	 * @param rightDist Degrees to spin the right wheel
    	 * @param power motor power to move at, from 0 to 1
    	 */
        public CmdMoveDistance(MoveEndMode endMode, double leftDist, double rightDist, double power, double timeout)
        {
        	super(timeout);
        	
        	this.power = power;
        	this.leftDist = leftDist;
        	this.rightDist = rightDist;
        	this.endMode = endMode;
        }

        protected void initialize()
        {
    		clearEncoders();
    		leftMotors.setTarget(RobotMath.sgn(leftDist) * power);
    		rightMotors.setTarget(RobotMath.sgn(rightDist) * power);
    		
    		Log.debug("CmdMoveDistance", "left: " + leftDist + ", right: " + rightDist);
        }

        // Make this return true when this Command no longer needs to run execute()
        protected boolean isFinished()
        {
        	boolean leftDone = leftDist == 0  || Math.abs(encLeft.getAngle()) >= Math.abs(leftDist);
        	boolean rightDone = rightDist == 0  || Math.abs(encRight.getAngle()) >= Math.abs(rightDist);
        	
        	if(isTimedOut())
        	{
        		return true;
        	}
        	
        	//Log.debug("CmdMoveDistance", "Left dst: " + encLeft.getDistanceInDegrees() + ", Right dst: " + encRight.getDistanceInDegrees());
        	switch(endMode)
        	{
        	case BOTH:
        		return leftDone && rightDone;
        	case EITHER:
        		return leftDone || rightDone;
        	case PER_SIDE:
        	default:
            	if(leftDone)
            	{
            		leftMotors.setTarget(0);
            	}
            	
            	if(rightDone)
            	{
            		rightMotors.setTarget(0);
            	}
        		return leftDone && rightDone;
        	}
        }

        // Called once after isFinished returns true
        protected void end()
        {
        	stopMovement();
        	if(isTimedOut())
        	{
    			AutoUtils.killRobot("Autonomous Move Overtime");
        	}
        }

        // Called when another command which requires one or more of the same
        // subsystems is scheduled to run
        protected void interrupted()
        {
        	stopMovement();
        }

		@Override
		protected void execute()
		{
			//do nothing
		}
    }
	
    /**
     * Command to to an arc turn in the specified amount of degrees.
     * 
     * Runs the opposite motors from the direction provided, so turning LEFT would set the RIGHT motors.
     * 
     * NOTE: currently requires that the front or back wheels be omni wheels for accurate turning.
     */
    public class CmdArcTurn extends CmdMoveDistance {
    	
    	/**
    	 * @param degs how far to turn in degrees.  Accepts negative values.
    	 * @param msec How long the move should take. If set to 0, do not time the move.
    	 */
        public CmdArcTurn(float degs, int msec, double power, Direction dir)
        {
        	super(MoveEndMode.PER_SIDE, 0, 0, power, msec);
        	
        	//this formula is explained on the info repository wiki
        	double wheelAngularDist = (2 * Math.PI * track) * (degs / 360);
        	
        	if(dir == Direction.RIGHT)
        	{
        		leftDist = wheelAngularDist;
        	}
        	else
        	{
        		rightDist = wheelAngularDist;
        	}
        }
    }
    
    /*
     *       /^\ 
     *      / _ \
     *     / | | \
     *    /  |_|  \
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
     * Command to stop the robot. Useful on bigger, heavier robots (like TheClawwww) which take longer to stop.
     */
    public class CmdBrake extends Command 
    {    	
    	VelocityPID leftPIDCalc, rightPIDCalc;
    	
        final static double completionThreshold = 20 * Angle.ROTATIONS;

    	/**
    	 * 
    	 * @param power Motor power to break with.
    	 * @param msec
    	 */
        public CmdBrake(PIDConstants brakingConstants, int msec)
        {
        	super(msec);
        	
        	leftPIDCalc = new VelocityPID(brakingConstants);
        	leftPIDCalc.setDesiredVelocity(0);
        	
        	rightPIDCalc = new VelocityPID(brakingConstants);
        	rightPIDCalc.setDesiredVelocity(0);
        }

        protected void initialize()
        {

        }

        // Called repeatedly when this Command is scheduled to run
        protected void execute()
        {
        	leftPIDCalc.update(encLeft.getAngularSpeed());
        	leftMotors.setTarget(leftPIDCalc.getOutput());
        	
        	rightPIDCalc.update(encRight.getAngularSpeed());
        	rightMotors.setTarget(rightPIDCalc.getOutput());
        }
        

        // Make this return true when this Command no longer needs to run execute()
        protected boolean isFinished()
        {
        	boolean leftFinished = Math.abs(encLeft.getAngularSpeed()) > completionThreshold;
        	boolean rightFinished = Math.abs(encRight.getAngularSpeed()) > completionThreshold;
        	
        	return leftFinished && rightFinished;
        }

        // Called once after isFinished returns true
        protected void end()
        {
    		leftPIDCalc.resetIntegral();
    		rightPIDCalc.resetIntegral();
        }

        // Called when another command which requires one or more of the same
        // subsystems is scheduled to run
        protected void interrupted()
        {
        	
        }
    }
    
    /**
     * Command to to an arc turn in the specified amount of degrees.
     * 
     * Sets the opposite motors from the direction provided, so turning LEFT would set the RIGHT motors.
     */
    public class CmdInPlaceTurn extends CmdMoveDistance 
    {
    	/**
    	 * @param degs how far to turn in degrees.  Accepts negative values.
    	 * @param msec How long the move should take. If set to 0, do not time the move
    	 */
        public CmdInPlaceTurn(float degs, int msec, Direction dir)
        {
        	this(degs, .5, msec, dir);
        }
        
        /**
    	 * @param degs how far to turn in degrees.  Accepts negative values.
    	 * @param msec How long the move should take. If set to 0, do not time the move
    	 */
        public CmdInPlaceTurn(float degs, double motorPower, int msec, Direction dir)
        {
        	//the encoder counts are an in-depth calculation, so we don't set them until after the super constructor
        	super(MoveEndMode.PER_SIDE, 0, 0, motorPower, msec);
        	
        	//this formula is explained in the info repository wiki
    		double wheelAngularDist = cmToEncDegrees(turningCircleCircumference*(degs/360.0)); 
        	
        	if(dir == Direction.RIGHT)
        	{
        		leftDist = wheelAngularDist;
        		rightDist = -wheelAngularDist;
        	}
        	else
        	{
        		leftDist = -wheelAngularDist;
        		rightDist = wheelAngularDist;
        	}
        }
    }
    
    
    /**
     * Command to move forward the given amount of centimeters.  Drives straight, if you've set up your 
     * speed multipliers properly.
     */
    public class CmdMoveForward extends CmdMoveDistance 
    {
    	
    	/**
    	 * @param d how far to move.  Accepts negative values.
    	 * @param msec How long the move should take. If set to 0, do not time the move
    	 */
        public CmdMoveForward(double d, int msec, boolean fullSpeed)
        {
        	super(MoveEndMode.PER_SIDE, cmToEncDegrees(d), cmToEncDegrees(d), fullSpeed ? 1 : .50, msec);
        }
        
    	/**
    	 * @param d how far to move.  Accepts negative values.
    	 * @param msec How long the move should take. If set to 0, do not time the move
    	 */
        public CmdMoveForward(double d, int msec, double power)
        {
        	super(MoveEndMode.PER_SIDE, cmToEncDegrees(d), cmToEncDegrees(d), power, msec);

        }
        
        @Override
        public void end()
        {
        	super.end();
    		Log.debug("CmdMoveForward", "The right side went " + ((encRight.getAngle() * 100.0) / encRight.getAngle()) + "% of the left side");
        }
    }
    
   /**
    * Command to move forward the given amount of centimeters.
    * 
    * It uses a feedback loop to make the robot drive straight.
    */
   public class CmdMoveStraightForward extends Command {

   	double _cm;
   	
   	int _msec;
   	
   	long startTime;
   	
   	double kP;
   	
   	double kD = .0001;
   	
   	double lastError = 0;
   	
   	/**
   	 * rotations that the move will take
   	 */
   	double enc;
   	
   	boolean rightDone = false;
   	
   	boolean leftDone = false;
   	
   	double pow;
   	
   	/**
   	 * @param d how far to move.  Accepts negative values.
   	 * @param kP The konstant of proportion.  Scales how the feedback affects the wheel speeds.
   	 * @param msec How long the move should take. If set to 0, do not time the move
   	 */
       public CmdMoveStraightForward(double d, double kP, int msec, double pow)
       {
       	_cm = d;
       	
       	_msec = msec;
       	
       	this.kP = kP;
       	
   		enc = abs(cmToEncDegrees(_cm));
   		int norm = (int) RobotMath.sgn(_cm);
   		this.pow = norm * pow;
       }

       protected void initialize()
       {
   		clearEncoders();
   		startTime = System.currentTimeMillis();
   		
   		leftMotors.setTarget(pow); //both sides start at the same power
   		rightMotors.setTarget(pow);
       }

       // Called repeatedly when this Command is scheduled to run
       protected void execute()
       {
       	//P calculation
       	double error = encLeft.getAngularSpeed() -  encRight.getAngularSpeed();
       	pow += kP * error;
       	pow += kD * lastError;
       	
       	lastError = error;
       	
   		rightMotors.setTarget(pow);

   		if(_msec != 0 && System.currentTimeMillis() - startTime >_msec)
   		{
   			stopMovement();
   			AutoUtils.killRobot("Move Overtime");
   		}
       }

       // Make this return true when this Command no longer needs to run execute()
       protected boolean isFinished()
       {
       	leftDone = Math.abs(encLeft.getAngle()) > enc;
       	rightDone = Math.abs(encRight.getAngle()) > enc;
       	
           return leftDone || rightDone;
       }

       // Called once after isFinished returns true
       protected void end()
       {
   		stopMovement();

       	Log.debug("CmdMoveStraightForward", "The right side went at " + ((encRight.getAngle() * 100.0) / encRight.getAngle()) + "% of the left side");
       }

       // Called when another command which requires one or more of the same
       // subsystems is scheduled to run
       protected void interrupted()
       {
       	
       }
   }
   
}