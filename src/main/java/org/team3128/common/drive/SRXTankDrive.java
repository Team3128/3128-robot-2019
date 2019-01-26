package org.team3128.common.drive;

import org.team3128.common.drive.routemaker.Routemaker;
import org.team3128.common.drive.routemaker.ProfilePoint;
import org.team3128.common.drive.routemaker.Waypoint;
import org.team3128.common.hardware.misc.TwoSpeedGearshift;
import org.team3128.common.util.Assert;
import org.team3128.common.util.Constants;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Angle;
import org.team3128.common.util.units.AngularSpeed;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.kauailabs.navx.frc.AHRS;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Class which represents a tank drive powered by Talon SRXs on a robot.
 *
 * Uses positional PID for high-accuracy autonomous moves. Make sure that the
 * SRXs have quadrature encoders attached, have FeedbackDevice set, and that are
 * configured to the correct number of counts per revolution.
 *
 * CALIBRATION PROCEDURE:
 * -------------------------------------------------------------------------------------------------
 * Get the PID constants to ballpark. EACH DAY OF EACH COMPETITION: - Run the
 * drive forward on the practice field for 100 inches at the speed that is used
 * for the competition autos - Adjust feedforward on each side until the robot
 * drives straight - Adjust the wheel diameter until the distance is correct to
 * the inch
 *
 * @author Jamie, Ronak
 *
 */
public class SRXTankDrive implements ITankDrive
{
	private TalonSRX leftMotors, rightMotors;

	public TalonSRX getLeftMotors() {
		return leftMotors;
	}

	public TalonSRX getRightMotors() {
		return rightMotors;
	}

	private TwoSpeedGearshift gearshift;

	/**
	 * The minimum speed (in RPM) of the wheels at which the robot should shift
	 * up to high gear if the robot was previously in low gear
	 */
	private double shiftUpSpeed;

	/**
	 * The maximum speed (in RPM) of the wheels at which the robot should shift
	 * down to low gear if the robot was previously in high gear
	 */
	private double shiftDownSpeed;

	/**
	 * How the drive should stop. Brake if the drive motors should apply power to
	 * stop the robot (typically set for teleop). Coast if the drive motors
	 * should not react when power is set to 0 (typically set for auto).
	 */
	private NeutralMode neutralMode;

	/**
	 * circumference of wheels in cm
	 */
	public final double wheelCircumfrence;

	/**
	 * horizontal distance between wheels in cm
	 */
	public final double wheelBase;

	/**
	 * Ratio between turns of the wheels to turns of the encoder
	 */
	private double gearRatio;

	/**
	 * The maxiumum measured speed of the drive motors, in native units
	 * per 100ms, of the robot driving on the ground at 100% throttle
	 */
	public int robotMaxSpeed;

	/**
	 * Speed scalar for the left and right wheels. Affects autonomous and
	 * teleop.
	 */
	private double leftSpeedScalar, rightSpeedScalar;

	public double getGearRatio()
	{
		return gearRatio;
	}

	public void setGearRatio(double gearRatio)
	{
		this.gearRatio = gearRatio;
	}

	// Singelton methods
	private static SRXTankDrive instance = null;

	public static SRXTankDrive getInstance() {
		if (instance != null) {
			return instance;
		}

		Log.fatal("SRXTankDrive", "Attempted to get instance before initializtion! Call initialize(...) first.");
		return null;
	}

	/**
	 * The "lead" Talon SRX on each drive side is the motor with a connected encoder. 
	 * Configure each non-leader Talon of both drive sides to follow their respective
	 * "lead" Talon using Follower mode.
	 *
	 * @param leftMotors
	 *            The "lead" Talon SRX on the left side.
	 * @param rightMotors
	 *            The "lead" Talon SRX on the right side.
	 * @param wheelCircumfrence
	 *            The circumference of the wheel
	 * @param gearRatio
	 *            The gear ratio of the turns of the wheels per turn of the
	 *            encoder shaft
	 * @param wheelBase
	 *            The distance between the front and back wheel on a side
	 * @param track
	 *            distance across between left and right wheels
	 * @param robotMaxSpeed
	 *            The lesser of the maxiumum measured speeds for both drive sides,
	 *            in native units per 100ms, of the robot driving on the ground at
	 *            100% throttle
	 */
	public static void initialize(TalonSRX leftMotors, TalonSRX rightMotors, double wheelCircumfrence, double gearRatio, double wheelBase, int robotMaxSpeed) {
		instance = new SRXTankDrive(leftMotors, rightMotors, wheelCircumfrence, gearRatio, wheelBase, robotMaxSpeed);
	}

	private SRXTankDrive(TalonSRX leftMotors, TalonSRX rightMotors, double wheelCircumfrence, double gearRatio, double wheelBase, int robotMaxSpeed)
	{
		this.leftMotors = leftMotors;
		this.rightMotors = rightMotors;

		this.wheelCircumfrence = wheelCircumfrence;
		this.wheelBase = wheelBase;
		this.gearRatio = gearRatio;
		this.robotMaxSpeed = robotMaxSpeed;

		leftSpeedScalar = 1;
		rightSpeedScalar = 1;

		if (gearRatio <= 0)
		{
			throw new IllegalArgumentException("Invalid gear ratio");
		}
	}

	private void setBrakeNeutralMode()
	{
		if (neutralMode != NeutralMode.Brake)
		{
			leftMotors.setNeutralMode(NeutralMode.Brake);
			rightMotors.setNeutralMode(NeutralMode.Brake);

			neutralMode = NeutralMode.Brake;
		}
	}

	private void setCoastNeutralMode()
	{
		leftMotors.setNeutralMode(NeutralMode.Coast);
		rightMotors.setNeutralMode(NeutralMode.Coast);

		neutralMode = NeutralMode.Coast;
	}

	// threshold below which joystick movements are ignored.
	final static double thresh = 0.2;

	/**
	 * Update the motor outputs with the given control values.
	 *
	 * @param joyX
	 *            horizontal control input
	 * @param joyY
	 *            vertical control input
	 * @param throttle
	 *            throttle control input scaled between 1 and -1 (-.8 is 10 %, 0
	 *            is 50%, 1.0 is 100%)
	 */
	@Override
	public void arcadeDrive(double joyX, double joyY, double throttle, boolean fullSpeed)
	{
		setBrakeNeutralMode();

		double spdL, spdR;

		if (!fullSpeed)
		{
			joyY *= .65;
		}
		else
		{
			joyY *= 1;
		}

		// scale from 1 to -1 to 1 to 0
		throttle = (throttle + 1) / 2;

		if (throttle < .3)
		{
			throttle = .3;
		}
		else if (throttle > .8)
		{
			throttle = 1;
		}

		joyY *= throttle;
		joyX *= throttle;

		spdR = RobotMath.clampPosNeg1(joyY + joyX);
		spdL = RobotMath.clampPosNeg1(joyY - joyX);

		// Log.debug("SRXTankDrive", "x1: " + joyX + " throttle: " + throttle +
		// " spdR: " + spdR + " spdL: " + spdL);

		leftMotors.set(ControlMode.PercentOutput, spdL);
		rightMotors.set(ControlMode.PercentOutput, spdR);
	}

	/**
	 * Set the left speed scalar. Must be between 0 and 1.
	 */
	public void setLeftSpeedScalar(double scalar)
	{
		Assert.inRange(scalar, 0, 1);
		leftSpeedScalar = scalar;
	}

	/**
	 * Set the right speed scalar. Must be between 0 and 1.
	 */
	public void setRightSpeedScalar(double scalar)
	{
		Assert.inRange(scalar, 0, 1);
		rightSpeedScalar = scalar;
	}

	/**
	 * Drive by providing motor powers for each side.
	 *
	 * @param powL
	 *            the left side power.
	 * @param powR
	 *            the right side power.
	 */
	public void tankDrive(double powL, double powR)
	{
		setBrakeNeutralMode();
		leftMotors.set(ControlMode.PercentOutput, powL);
		rightMotors.set(ControlMode.PercentOutput, powR);
	}

	public void clearEncoders()
	{
		leftMotors.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
		rightMotors.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
	}

	@Override
	public void stopMovement()
	{
		setBrakeNeutralMode();
		tankDrive(0, 0);
	}

	/**
	 * Adds a two-speed gearshift with auto-shifting to the robot drive.
	 *
	 * @param gearshift
	 *            Two-speed gearshift object
	 * @param shiftUpSpeed
	 *            The minimum speed (in RPM) for which the gearshift should
	 *            shift to high gear
	 * @param shiftDownSpeed
	 *            The maximum speed (in RPM) for which the gearshift should
	 *            shift to low gear
	 */
	public void addShifter(TwoSpeedGearshift gearshift, double shiftUpSpeed, double shiftDownSpeed)
	{
		this.gearshift = gearshift;
		this.shiftUpSpeed = shiftUpSpeed;
		this.shiftDownSpeed = shiftDownSpeed;
	}

	public void shiftToHigh()
	{
		if (gearshift != null)
		{
			gearshift.shiftToHigh();
		}
		else
		{
			Log.info("SRXTankDrive", "You can't shift gears. The robot doesn't actually have a gearshift.");
		}
	}

	public void shiftToLow()
	{
		if (gearshift != null)
		{
			gearshift.shiftToLow();
		}
		else
		{
			Log.info("SRXTankDrive", "You can't shift gears. The robot doesn't actually have a gearshift.");
		}
	}

	public void shift()
	{
		if (gearshift != null)
		{
			gearshift.shiftToOtherGear();
		}
		else
		{
			Log.info("SRXTankDrive", "You can't shift gears. The robot doesn't actually have a gearshift.");
		}
	}

	public boolean isInHighGear()
	{
		if (gearshift != null)
		{
			return gearshift.isInHighGear();
		}
		else
		{
			Log.fatal("SRXTankDrive",
					"There is only one gear. The robot doesn't actually have a gearshift.");
			return false;
		}
	}

	public void autoshift()
	{
		if (gearshift != null)
		{
			double rightSpeed = rightMotors.getSelectedSensorVelocity(0) / AngularSpeed.NATIVE_UNITS_PER_100MS;
			double leftSpeed = leftMotors.getSelectedSensorVelocity(0) / AngularSpeed.NATIVE_UNITS_PER_100MS;

			if (gearshift.isInHighGear() && (rightSpeed < 0 && leftSpeed > 0) || (rightSpeed > 0 && leftSpeed < 0))
			{
				gearshift.shiftToLow();
			}
			else if (!gearshift.isInHighGear() && (rightSpeed > shiftUpSpeed && leftSpeed > shiftUpSpeed))
			{
				gearshift.shiftToHigh();
			}
			else if (gearshift.isInHighGear() && (rightSpeed < shiftDownSpeed && leftSpeed < shiftDownSpeed))
			{
				gearshift.shiftToLow();
			}
		}
		else
		{
			Log.info("SRXTankDrive", "You can't shift gears. The robot doesn't actually have a gearshift.");
		}
	}

	/**
	 * Get the estimated angle that the robot has turned since the encoders were
	 * last reset, based on the relative distances of each side.
	 *
	 * Range: [0, 360) 0 degrees is straight ahead.
	 *
	 * @return
	 */
	public double getRobotAngle()
	{
		double leftDist = encDistanceToCm(leftMotors.getSelectedSensorPosition(0) * Angle.ROTATIONS);
		double rightDist = encDistanceToCm(rightMotors.getSelectedSensorPosition(0) * Angle.ROTATIONS);

		double difference = leftDist - rightDist;

		return RobotMath.normalizeAngle((difference / (Math.PI * wheelBase)) * Angle.ROTATIONS);
	}

	public class CmdStaticRouteDrive extends CmdMotionProfileMove {
		private MotionProfileStatus leftStatus, rightStatus;
		private Waypoint[] waypoints;
		private double power;

		private Notifier processNotifier;

		public CmdStaticRouteDrive(double power, double timeoutMs, Waypoint... waypoints) {
			super(timeoutMs);

			this.power = power;
			this.waypoints = waypoints;

			leftStatus = new MotionProfileStatus();
			leftStatus.isLast = false;
			rightStatus = new MotionProfileStatus();
			rightStatus.isLast = false;

			processNotifier = new Notifier(() -> {
				System.out.println("Processing");
				leftMotors.processMotionProfileBuffer();
				rightMotors.processMotionProfileBuffer();
			});
		}

		@Override
		protected void initialize() {
			super.initialize();

			leftMotors.changeMotionControlFramePeriod((int) (Routemaker.durationMs / 2.3));
			rightMotors.changeMotionControlFramePeriod((int) (Routemaker.durationMs / 2.3));

			Routemaker rm = new Routemaker(power, waypoints);

			double speed;

			TrajectoryPoint trajPoint = new TrajectoryPoint();
			trajPoint.profileSlotSelect0 = 0;
			trajPoint.zeroPos = false;
			trajPoint.isLastPoint = false;

			ProfilePoint profilePoint;

			boolean first = true;

			do {
				speed = 1.0;
				if (rm.s < 0.4) {
					speed = (0.1 + rm.s) / 0.5;
				}
				else if (rm.s > 0.6) {
					speed = (1.1 - rm.s) / 0.5;
				}

				profilePoint = rm.getNextPoint(speed);
				//System.out.println(profilePoint.x + "," + profilePoint.y + " (" + profilePoint.durationMs + ")");

				if (profilePoint.last)
					trajPoint.isLastPoint = true;

				trajPoint.timeDur = profilePoint.durationMs;

				trajPoint.position = profilePoint.leftDistance;
				trajPoint.velocity = profilePoint.leftSpeed;
				leftMotors.pushMotionProfileTrajectory(trajPoint);

				trajPoint.position = profilePoint.rightDistance;
				trajPoint.velocity = profilePoint.rightSpeed;
				rightMotors.pushMotionProfileTrajectory(trajPoint);

				if (first) {
					processNotifier.startPeriodic(Routemaker.durationSec / 2);

					leftMotors.set(ControlMode.MotionProfile, 1);
					rightMotors.set(ControlMode.MotionProfile, 1);

					first = false;
				}

			} while (!profilePoint.last);
		}

		@Override
		protected synchronized boolean isFinished() {
			leftMotors.getMotionProfileStatus(leftStatus);
			rightMotors.getMotionProfileStatus(rightStatus);

			System.out.println(leftStatus.topBufferCnt + " " + leftStatus.btmBufferCnt);

			return super.isFinished() || leftStatus.isLast && rightStatus.isLast;
		}

		@Override
		protected synchronized void end() {
			super.end();
			processNotifier.close();
			Log.info("CmdStaticRouteDrive", "Finished.");
		}
	}

	public abstract class CmdMotionProfileMove extends Command {
		public CmdMotionProfileMove(double timeoutMs) {
			super(timeoutMs / 1000.0);
		}

		protected void initialize() {
			leftMotors.clearMotionProfileHasUnderrun(Constants.CAN_TIMEOUT);
			rightMotors.clearMotionProfileHasUnderrun(Constants.CAN_TIMEOUT);

			leftMotors.clearMotionProfileTrajectories();
			rightMotors.clearMotionProfileTrajectories();

			leftMotors.configMotionProfileTrajectoryPeriod(0, Constants.CAN_TIMEOUT);
			rightMotors.configMotionProfileTrajectoryPeriod(0, Constants.CAN_TIMEOUT);

			leftMotors.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
			rightMotors.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
		}

		@Override
		protected void execute() {
		}

		@Override
		protected boolean isFinished() {
			return this.isTimedOut();
		}

		@Override
		protected void end() {
			leftMotors.clearMotionProfileTrajectories();
			rightMotors.clearMotionProfileTrajectories();
		}
	}

	/**
	 * Convert cm of robot movement to encoder movement in degrees
	 *
	 * @param cm
	 * @return
	 */
	public double cmToEncDegrees(double cm)
	{
		return (cm * 360) / (wheelCircumfrence * gearRatio);
	}

	/**
	 * Convert cm of robot movement to encoder rotations
	 *
	 * @param cm
	 * @return
	 */
	public double encDistanceToCm(double encDistance)
	{
		return (encDistance / 360) * wheelCircumfrence * gearRatio;
	}

	/**
	 * Enum for how CmdMoveDistance determines when to end a move command.
	 *
	 * @author Jamie
	 *
	 */
	public enum MoveEndMode
	{
		BOTH, // ends when both sides have reached their targets.
		EITHER, // Stops both sides when either side has reached its target.
				// Force stops the move command of the slower side.
	}

	/**
	 * Command to move each side of the drivetrain a specified distance, using the MotionMagic control mode.
	 *
	 * Common logic shared by all of the autonomous movement commands
	 */
	public class CmdMotionMagicMove extends Command
	{
		// when the wheels' angular distance get within this threshold of the
		// correct value, that side is considered done
		final static double MOVEMENT_ERROR_THRESHOLD = 70 * Angle.DEGREES;

		protected double power;

		protected double leftAngle, rightAngle;

		protected int correctDistanceCount = 0;

		protected MoveEndMode endMode;

		boolean leftDone;
		boolean rightDone;

		boolean useScalars;

		/**
		 * @param endMode - The MoveEndMode for this command.
		 * @param leftAngle - Degrees to rotate the left wheel
		 * @param rightAngle - Degrees to rotate the right wheel
		 * @param power - fractional power to move drive wheels at from 0 to 1
		 * @param useScalars - whether or not to scale output left and right powers
		 * @param timeoutMs - The maximum time (in milliseconds) to run the command for
		 */
		public CmdMotionMagicMove(MoveEndMode endMode, double leftAngle, double rightAngle, double power, boolean useScalars,
				double timeoutMs)
		{
			super(timeoutMs / 1000.0);

			this.power = power;
			this.leftAngle = leftAngle;
			this.rightAngle = rightAngle;

			this.endMode = endMode;
			this.useScalars = useScalars;
		}

		protected void initialize()
		{
			Log.info("CmdMotionMagicMove", "Initializing...");

			clearEncoders();
			setCoastNeutralMode();

			// Coast speed measured in nu/100ms
			double leftSpeed = (robotMaxSpeed * power * ((useScalars) ? leftSpeedScalar : 1.0));
			double rightSpeed = (robotMaxSpeed * power * ((useScalars) ? rightSpeedScalar : 1.0));

			if (leftAngle == 0) {
				leftSpeed = 0;
			}
			else if (rightAngle == 0) {
				rightSpeed = 0;
			}
			else if (Math.abs(leftAngle) > Math.abs(rightAngle)) {
				rightSpeed *= Math.abs(rightAngle / leftAngle);
			}
			else if (Math.abs(leftAngle) < Math.abs(rightAngle)) {
				leftSpeed *= Math.abs(leftAngle / rightAngle);
			}

			ControlMode leftMode = ControlMode.MotionMagic;
			ControlMode rightMode = ControlMode.MotionMagic;

			if (useScalars)
			{
				Log.info("CmdMotionMagicMove", "Using scalars.");
			}

			// motion magic does not work well when the distance is 0
			if (leftAngle == 0)
			{
				leftMode = ControlMode.Position;
			}
			if (rightAngle == 0)
			{
				rightMode = ControlMode.Position;
			}

			leftMotors.configMotionCruiseVelocity((int) leftSpeed, Constants.CAN_TIMEOUT);
			leftMotors.configMotionAcceleration((int) (leftSpeed / 2), Constants.CAN_TIMEOUT);

			leftMotors.set(leftMode, leftAngle / Angle.CTRE_MAGENC_NU);

			rightMotors.configMotionCruiseVelocity((int) rightSpeed, Constants.CAN_TIMEOUT);
			rightMotors.configMotionAcceleration((int) (rightSpeed / 2), Constants.CAN_TIMEOUT);

			rightMotors.set(rightMode, rightAngle / Angle.CTRE_MAGENC_NU);

			Log.debug("CmdMotionMagicMove",
				  "\n"
				+ "  Distances\n"
				+ "    L: " + leftAngle / Angle.CTRE_MAGENC_NU  + " rot\n"
				+ "    R: " + rightAngle / Angle.CTRE_MAGENC_NU + " rot\n"
				+ "  Speeds\n"
				+ "    L: " + leftSpeed  + " RPM"
				+ "    R: " + rightSpeed + " RPM"
			);

			try
			{
				Thread.sleep(100);
			}
			catch (InterruptedException e)
			{
				e.printStackTrace();
			}
		}

		// Make this return true when this Command no longer needs to run
		// execute()
		protected boolean isFinished()
		{
			double leftError = leftMotors.getSelectedSensorPosition(0) * Angle.CTRE_MAGENC_NU - leftAngle;
			double rightError = rightMotors.getSelectedSensorPosition(0) * Angle.CTRE_MAGENC_NU - rightAngle;

			Log.debug("CmdMotionMagicMove",
					  "L=" + leftMotors.getSelectedSensorPosition(0)  + "nu; err=" + leftError  + "deg, "
					+ "R=" + rightMotors.getSelectedSensorPosition(0) + "nu; err=" + rightError + "deg.");

			leftDone = leftAngle == 0 || Math.abs(leftError) < MOVEMENT_ERROR_THRESHOLD;
			rightDone = rightAngle == 0 || Math.abs(rightError) < MOVEMENT_ERROR_THRESHOLD;

			if (isTimedOut())
			{
				Log.unusual("CmdMotionMagicMove", "Autonomous Move Overtime.");
				return true;
			}

			boolean isInZone;

			switch (endMode)
			{
			case BOTH:
				isInZone = leftDone && rightDone;
				break;
			case EITHER:
			default:
				isInZone = leftDone || rightDone;
				break;
			}

			if (isInZone)
			{
				++correctDistanceCount;
			}
			else
			{
				correctDistanceCount = 0;
			}

			return correctDistanceCount > 25;

		}

		// Called once after isFinished returns true
		protected void end()
		{
			Log.info("CmdMotionMagicMove", "Ending normally.");

			stopMovement();
		}

		// Called when another command which requires one or more of the same
		// subsystems is scheduled to run
		protected void interrupted()
		{
			Log.info("CmdMotionMagicMove", "Interrupted.");

			stopMovement();
		}
	}

	/**
	 * Command to move forward the given amount of centimeters. Drives straight,
	 * if you've set up your speed multipliers properly.
	 */
	public class CmdDriveStraight extends CmdMotionMagicMove
	{
		/**
		 * @param distance - The linear distance to drive forward
		 * @param power - fractional power to move drive wheels at from 0 to 1
		 * @param timeoutMs - The maximum time to run the move (in milliseconds)
		 */
		public CmdDriveStraight(double distance, double power, int timeoutMs)
		{
			super(MoveEndMode.BOTH, cmToEncDegrees(distance), cmToEncDegrees(distance), power, false, timeoutMs);

		}
	}

	/**
	 * Command to turn in an arc with a certain radius for the specified amount
	 * of degrees.
	 *
	 * Unlike standard arc turn, it doesn't actually require omni wheels because
	 * the slippage is even less than an in place turn, so it's actually more accurate.
	 *
	 * Try to use this as frequently as you can, unless another method is quicker.
	 */
	public class CmdArcTurn extends CmdMotionMagicMove
	{
		/**
		 * @param radius - The radius that the robot should drive the arc at.
		 * @param angle - The distance to turn in degrees. Accepts negative values.
		 * @param dir - The direction to turn towards.
		 * @param power - The fractional power to drive the robot (from 0 to 1)
		 * @param timeoutMs - The maximum time to run the move (in milliseconds)
		 */
		public CmdArcTurn(double radius, float angle, Direction dir, double power, int timeoutMs)
		{
			super(MoveEndMode.BOTH, 0, 0, power, false, timeoutMs);

			// this formula is explained on the info repository wiki
			double innerAngularDist = cmToEncDegrees((angle * Math.PI / 180.0) * (radius - 0.5 * wheelBase));
			double outerAngularDist = cmToEncDegrees((angle * Math.PI / 180.0) * (radius + 0.5 * wheelBase));

			if (dir == Direction.RIGHT)
			{
				rightAngle = innerAngularDist;
				leftAngle = outerAngularDist;
			}
			else
			{
				rightAngle = outerAngularDist;
				leftAngle = innerAngularDist;
			}
		}
	}

	/**
	 * Command to to an arc turn in the specified amount of degrees.
	 *
	 * Sets the opposite motors from the direction provided, so turning LEFT
	 * would set the RIGHT motors.
	 */
	public class CmdInPlaceTurn extends CmdMotionMagicMove
	{
		/**
		 * @param angle - The angle to turn in degrees. Accepts negative values.
		 * @param dir - The direction to turn towards.
		 * @param power - The relative speed to drive the turn at (from 0 to 1)
		 * @param timeoutMs - The maximum time to run the move (in milliseconds)
		 */
		public CmdInPlaceTurn(double angle, Direction dir, double power, int timeoutMs)
		{
			// the encoder counts are an in-depth calculation, so we don't set
			// them until after the super constructor
			super(MoveEndMode.BOTH, 0, 0, power, false, timeoutMs);

			// this formula is explained in the info repository wiki
			double wheelAngularDist = cmToEncDegrees((Math.PI * wheelBase) * (angle / 360.0));

			if (dir == Direction.RIGHT)
			{
				leftAngle = wheelAngularDist;
				rightAngle = -wheelAngularDist;
			}
			else
			{
				leftAngle = -wheelAngularDist;
				rightAngle = wheelAngularDist;
			}
		}
	}

	/**
	 * Callibration command to determine effective wheelbase of the robot. This is to account for the field material scrubbing against the wheels,
	 * resisting a turning motion.
	 *
	 * The effective wheelbase should always be larger than the measured wheelbase, but it will differ by a different factor for each
	 * robot. For instance, a 6-wheel tank drive with corner omnis will have a relatively small factor of difference, while a tank drive
	 * with pneumatic wheels will have a very large difference.
	 */
	public class CmdDetermineWheelbase extends Command {
		double leftSpeed, rightSpeed;

		AHRS ahrs;

		double leftDistance, rightDistance;

		double theta0, theta1;
		double dTheta;

		double leftWheelbase, rightWheelbase;

		Double calculatedWheelbase;

		/**
		 * @param durationMs - The amount of time for which the robot should coast.
		 * @param leftSpeed - The coast velocity of the left drive wheels.
		 * @param rightSpeed - The coast velocity of the right drive wheels.
		 * @param calculatedWheelbase - The Double wrapper object that the finished command should stick the calculated wheelbase in.
		 */
		public CmdDetermineWheelbase(double durationMs, double leftSpeed, double rightSpeed, Double calculatedWheelbase) {
			super(durationMs);

			this.leftSpeed = leftSpeed;
			this.rightSpeed = rightSpeed;
		}

		protected void initialize() {
			for(int i = 0; i <= 10000 ; i++) {
				getLeftMotors().set(ControlMode.Velocity, leftSpeed * i/10000);
				getRightMotors().set(ControlMode.Velocity, rightSpeed * i/10000);
			}

			getLeftMotors().setSelectedSensorPosition(0);
			getRightMotors().setSelectedSensorPosition(0);

			theta0 = ahrs.getAngle()*(Math.PI/180);
		}

		@Override
		protected boolean isFinished() {
			return this.isTimedOut();
		}

		@Override
		protected void end() {
			theta1 = ahrs.getAngle()*(Math.PI/180);
			dTheta = theta1 - theta0;

			leftDistance = (wheelCircumfrence * getLeftMotors().getSelectedSensorPosition() / 4096);
			rightDistance = (wheelCircumfrence * getRightMotors().getSelectedSensorPosition() / 4096);

			stopMovement();

			leftWheelbase = 2 * (leftDistance/dTheta) - 2 * (leftDistance + rightDistance) / (2 * dTheta);
			rightWheelbase = -2 * (rightDistance/dTheta) + 2 * (leftDistance + rightDistance) / (2 * dTheta);

			calculatedWheelbase = (leftWheelbase + rightWheelbase)/2;
		}
	}
}