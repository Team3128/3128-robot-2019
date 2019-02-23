package org.team3128.common.drive;

import edu.wpi.first.wpilibj.DriverStation;
import org.team3128.common.drive.routemaker.Routemaker;
import org.team3128.common.drive.routemaker.ProfilePoint;
import org.team3128.common.drive.routemaker.Waypoint;
import org.team3128.common.hardware.misc.TwoSpeedGearshift;
import org.team3128.common.hardware.navigation.Gyro;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.util.Assert;
import org.team3128.common.util.Constants;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.datatypes.PIDConstants;
import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Angle;
import org.team3128.common.util.units.AngularSpeed;
import org.team3128.common.util.units.Length;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
public class SRXTankDrive implements ITankDrive {
	private TalonSRX leftMotors, rightMotors;

	public TalonSRX getLeftMotors() {
		return leftMotors;
	}

	public TalonSRX getRightMotors() {
		return rightMotors;
	}

	private TwoSpeedGearshift gearshift;

	/**
	 * The minimum speed (in RPM) of the wheels at which the robot should shift up
	 * to high gear if the robot was previously in low gear
	 */
	private double shiftUpSpeed;

	/**
	 * The maximum speed (in RPM) of the wheels at which the robot should shift down
	 * to low gear if the robot was previously in high gear
	 */
	private double shiftDownSpeed;

	/**
	 * circumference of wheels in cm
	 */
	public final double wheelCircumfrence;

	/**
	 * horizontal distance between wheels in cm
	 */
	public final double wheelBase;

	// /**
	//  * Ratio between turns of the wheels to turns of the encoder
	//  */
	// private double gearRatio;

	/**
	 * The maxiumum measured speed of the drive motors, in native units per 100ms,
	 * of the robot driving on the ground at 100% throttle
	 */
	public int robotMaxSpeed;

	/**
	 * Speed scalar for the left and right wheels. Affects autonomous and teleop.
	 */
	private double leftSpeedScalar, rightSpeedScalar;

	/**
	 * Callbacks to be executed when the robot is switched between
	 * operator control and autonomous control to ensure all drive
	 * motors are driving forward.
	 */
	private SRXInvertCallback teleopInvertCallback, autoInvertCallback;
	private enum DriveMode {
		TELEOP(NeutralMode.Coast),
		AUTONOMOUS(NeutralMode.Brake);

		private NeutralMode neutralMode;

		private DriveMode(NeutralMode neutralMode) {
			this.neutralMode = neutralMode;
		}

		public NeutralMode getNeutralMode() {
			return neutralMode;
		}
	}
	private DriveMode driveMode;


	/**
	 * FPID constants for both left and righ drive sides in both
	 * Motion Magic/Motion Profile control mode as well as velocity
	 * control mode.
	 */
	private PIDConstants leftMotionProfilePID, leftVelocityPID;
	private PIDConstants rightMotionProfilePID, rightVelocityPID;

	// public double getGearRatio()
	// {
	// 	return gearRatio;
	// }

	// public void setGearRatio(double gearRatio)
	// {
	// 	this.gearRatio = gearRatio;
	// }

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
	 * The "lead" Talon SRX on each drive side is the motor with a connected
	 * encoder. Configure each non-leader Talon of both drive sides to follow their
	 * respective "lead" Talon using Follower mode.
	 *
	 * @param leftMotors
	 *            The "lead" Talon SRX on the left side.
	 * @param rightMotors
	 *            The "lead" Talon SRX on the right side.
	 * @param wheelCircumfrence
	 *            The amount of units of length the robot travels per rotation of the encoder stage
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
	public static void initialize(TalonSRX leftMotors, TalonSRX rightMotors, double wheelCircumfrence, double wheelBase, int robotMaxSpeed, SRXInvertCallback driveInverts) {
		instance = new SRXTankDrive(leftMotors, rightMotors, wheelCircumfrence, wheelBase, robotMaxSpeed, driveInverts);
	}

	private SRXTankDrive(TalonSRX leftMotors, TalonSRX rightMotors, double wheelCircumfrence, double wheelBase, int robotMaxSpeed, SRXInvertCallback driveInverts)
	{
		this.leftMotors = leftMotors;
		this.rightMotors = rightMotors;

		this.wheelCircumfrence = wheelCircumfrence;
		this.wheelBase = wheelBase;
		//this.gearRatio = gearRatio;
		this.robotMaxSpeed = robotMaxSpeed;

		leftSpeedScalar = 1;
		rightSpeedScalar = 1;

		driveInverts.invertMotors();

		configureDriveMode(DriveMode.TELEOP);

		loadSRXPIDConstants();
		setupDashboardPIDListener();

		sendPIDConstants();

		// if (gearRatio <= 0)
		// {
		// 	throw new IllegalArgumentException("Invalid gear ratio");
		// }
	}

	private void configureDriveMode(DriveMode mode) {
		if (driveMode != mode) {			
			leftMotors.setNeutralMode(mode.getNeutralMode());
			rightMotors.setNeutralMode(mode.getNeutralMode());

			driveMode = mode;
		}
	}

	// threshold below which joystick movements are ignored.
	final static double thresh = 0.2;

	/**
	 * Update the motor outputs with the given control values.
	 *
	 * @param joyX     horizontal control input
	 * @param joyY     vertical control input
	 * @param throttle throttle control input scaled between 1 and -1 (-.8 is 10 %,
	 *                 0 is 50%, 1.0 is 100%)
	 */
	@Override
	public void arcadeDrive(double joyX, double joyY, double throttle, boolean fullSpeed)
	{
		configureDriveMode(DriveMode.TELEOP);

		double spdL, spdR;

		if (!fullSpeed) {
			joyY *= .65;
		} else {
			joyY *= 1;
		}

		// scale from 1 to -1 to 1 to 0
		throttle = (throttle + 1) / 2;

		if (throttle < .3) {
			throttle = .3;
		} else if (throttle > .8) {
			throttle = 1;
		}

		joyY *= throttle;
		joyX *= throttle;

		spdR = rightSpeedScalar * RobotMath.clampPosNeg1(joyY + joyX);
		spdL = leftSpeedScalar  * RobotMath.clampPosNeg1(joyY - joyX);

		leftMotors.set(ControlMode.PercentOutput, spdL);
		rightMotors.set(ControlMode.PercentOutput, spdR);
	}

	/**
	 * Set the left speed scalar. Must be between 0 and 1.
	 */
	public void setLeftSpeedScalar(double scalar) {
		Assert.inRange(scalar, 0, 1);
		leftSpeedScalar = scalar;
	}

	/**
	 * Set the right speed scalar. Must be between 0 and 1.
	 */
	public void setRightSpeedScalar(double scalar) {
		Assert.inRange(scalar, 0, 1);
		rightSpeedScalar = scalar;
	}

	/**
	 * Drive by providing motor powers for each side.
	 *
	 * @param powL the left side power.
	 * @param powR the right side power.
	 */
	public void tankDrive(double powL, double powR) {
		configureDriveMode(DriveMode.TELEOP);

		leftMotors.set(ControlMode.PercentOutput, powL);
		rightMotors.set(ControlMode.PercentOutput, powR);
	}

	public void clearEncoders() {
		leftMotors.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
		rightMotors.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
	}

	@Override
	public void stopMovement() {
		configureDriveMode(DriveMode.AUTONOMOUS);

		tankDrive(0, 0);
	}

	/**
	 * Adds a two-speed gearshift with auto-shifting to the robot drive.
	 *
	 * @param gearshift      Two-speed gearshift object
	 * @param shiftUpSpeed   The minimum speed (in RPM) for which the gearshift
	 *                       should shift to high gear
	 * @param shiftDownSpeed The maximum speed (in RPM) for which the gearshift
	 *                       should shift to low gear
	 */
	public void addShifter(TwoSpeedGearshift gearshift, double shiftUpSpeed, double shiftDownSpeed) {
		this.gearshift = gearshift;
		this.shiftUpSpeed = shiftUpSpeed;
		this.shiftDownSpeed = shiftDownSpeed;
	}

	public void shiftToHigh() {
		if (gearshift != null) {
			gearshift.shiftToHigh();
		} else {
			Log.info("SRXTankDrive", "You can't shift gears. The robot doesn't actually have a gearshift.");
		}
	}

	public void shiftToLow() {
		if (gearshift != null) {
			gearshift.shiftToLow();
		} else {
			Log.info("SRXTankDrive", "You can't shift gears. The robot doesn't actually have a gearshift.");
		}
	}

	public void shift() {
		if (gearshift != null) {
			gearshift.shiftToOtherGear();
		} else {
			Log.info("SRXTankDrive", "You can't shift gears. The robot doesn't actually have a gearshift.");
		}
	}

	public boolean isInHighGear() {
		if (gearshift != null) {
			return gearshift.isInHighGear();
		} else {
			Log.fatal("SRXTankDrive", "There is only one gear. The robot doesn't actually have a gearshift.");
			return false;
		}
	}

	public void autoshift() {
		if (gearshift != null) {
			double rightSpeed = rightMotors.getSelectedSensorVelocity(0) / AngularSpeed.NATIVE_UNITS_PER_100MS;
			double leftSpeed = leftMotors.getSelectedSensorVelocity(0) / AngularSpeed.NATIVE_UNITS_PER_100MS;

			if (gearshift.isInHighGear() && (rightSpeed < 0 && leftSpeed > 0) || (rightSpeed > 0 && leftSpeed < 0)) {
				gearshift.shiftToLow();
			} else if (!gearshift.isInHighGear() && (rightSpeed > shiftUpSpeed && leftSpeed > shiftUpSpeed)) {
				gearshift.shiftToHigh();
			} else if (gearshift.isInHighGear() && (rightSpeed < shiftDownSpeed && leftSpeed < shiftDownSpeed)) {
				gearshift.shiftToLow();
			}
		} else {
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
	public double getRobotAngle() {
		double leftDist = encDistanceToCm(leftMotors.getSelectedSensorPosition(0) * Angle.ROTATIONS);
		double rightDist = encDistanceToCm(rightMotors.getSelectedSensorPosition(0) * Angle.ROTATIONS);

		double difference = leftDist - rightDist;

		return RobotMath.normalizeAngle((difference / (Math.PI * wheelBase)) * Angle.ROTATIONS);
	}

	/**
	 * Loads stored PID constants from the drive Talon SRXs.
	 */
	public void loadSRXPIDConstants() {
		SlotConfiguration configs = new SlotConfiguration();

		leftMotors.getSlotConfigs(configs, 0, Constants.CAN_TIMEOUT);
		leftMotionProfilePID = new PIDConstants(configs.kF, configs.kP, configs.kI, configs.kD);
		Log.info("SRXTankDrive", "Left MP: " + leftMotionProfilePID);

		leftMotors.getSlotConfigs(configs, 1, Constants.CAN_TIMEOUT);
		leftVelocityPID = new PIDConstants(configs.kF, configs.kP, configs.kI, configs.kD);
		Log.info("SRXTankDrive", "Left V: " + leftVelocityPID);

		rightMotors.getSlotConfigs(configs, 0, Constants.CAN_TIMEOUT);
		rightMotionProfilePID = new PIDConstants(configs.kF, configs.kP, configs.kI, configs.kD);
		Log.info("SRXTankDrive", "Right MP: " + rightMotionProfilePID);

		rightMotors.getSlotConfigs(configs, 1, Constants.CAN_TIMEOUT);
		rightVelocityPID = new PIDConstants(configs.kF, configs.kP, configs.kI, configs.kD);
		Log.info("SRXTankDrive", "Right V: " + rightVelocityPID);
	}

	public void setLeftPID() {
		Log.info("SRXTankDrive", "Setting left PID constants.");

		leftMotors.config_kF(0, leftMotionProfilePID.kF);
		leftMotors.config_kP(0, leftMotionProfilePID.kP);
		leftMotors.config_kI(0, leftMotionProfilePID.kI);
		leftMotors.config_kD(0, leftMotionProfilePID.kD);

		leftMotors.config_kF(1, leftVelocityPID.kF);
		leftMotors.config_kP(1, leftVelocityPID.kP);
		leftMotors.config_kI(1, leftVelocityPID.kI);
		leftMotors.config_kD(1, leftVelocityPID.kD);

		sendPIDConstants();
	}

	public void setRightPID() {
		Log.info("SRXTankDrive", "Setting right PID constants.");

		rightMotors.config_kF(0, rightMotionProfilePID.kF);
		rightMotors.config_kP(0, rightMotionProfilePID.kP);
		rightMotors.config_kI(0, rightMotionProfilePID.kI);
		rightMotors.config_kD(0, rightMotionProfilePID.kD);

		rightMotors.config_kF(1, rightVelocityPID.kF);
		rightMotors.config_kP(1, rightVelocityPID.kP);
		rightMotors.config_kI(1, rightVelocityPID.kI);
		rightMotors.config_kD(1, rightVelocityPID.kD);

		sendPIDConstants();
	}

	/**
	 * Sets up listeners to update drive PID constants when sent from
	 * NarwhalDashboard
	 */
	public void setupDashboardPIDListener() {
		NarwhalDashboard.addNumDataListener("leftPID", (double constants[]) -> {
			this.leftMotionProfilePID.kF = constants[0];
			this.leftMotionProfilePID.kP = constants[1];
			this.leftMotionProfilePID.kI = constants[2];
			this.leftMotionProfilePID.kD = constants[3];

			this.leftVelocityPID.kF = constants[0];
			this.leftVelocityPID.kP = constants[4];
			this.leftVelocityPID.kI = constants[5];
			this.leftVelocityPID.kD = constants[6];

			this.setLeftPID();
		});

		NarwhalDashboard.addNumDataListener("rightPID", (double constants[]) -> {
			this.rightMotionProfilePID.kF = constants[0];
			this.rightMotionProfilePID.kP = constants[1];
			this.rightMotionProfilePID.kI = constants[2];
			this.rightMotionProfilePID.kD = constants[3];

			this.rightVelocityPID.kF = constants[0];
			this.rightVelocityPID.kP = constants[4];
			this.rightVelocityPID.kI = constants[5];
			this.rightVelocityPID.kD = constants[6];

			this.setRightPID();
		});
	}

	/**
	 * Sends PID constants to NarwhalDashboard
	 */
	public void sendPIDConstants() {
		NarwhalDashboard.put("l_f", leftMotionProfilePID.kF);

		NarwhalDashboard.put("l_mp_p", leftMotionProfilePID.kP);
		NarwhalDashboard.put("l_mp_i", leftMotionProfilePID.kI);
		NarwhalDashboard.put("l_mp_d", leftMotionProfilePID.kD);

		NarwhalDashboard.put("l_v_p", leftVelocityPID.kP);
		NarwhalDashboard.put("l_v_i", leftVelocityPID.kI);
		NarwhalDashboard.put("l_v_d", leftVelocityPID.kD);


		NarwhalDashboard.put("r_f", rightMotionProfilePID.kF);

		NarwhalDashboard.put("r_mp_p", rightMotionProfilePID.kP);
		NarwhalDashboard.put("r_mp_i", rightMotionProfilePID.kI);
		NarwhalDashboard.put("r_mp_d", rightMotionProfilePID.kD);

		NarwhalDashboard.put("r_v_p", leftVelocityPID.kP);
		NarwhalDashboard.put("r_v_i", leftVelocityPID.kI);
		NarwhalDashboard.put("r_v_d", leftVelocityPID.kD);
	}

	/**
	 * Convert cm of robot movement to encoder movement in degrees
	 *
	 * @param cm
	 * @return
	 */
	public double cmToEncDegrees(double cm)
	{
		return (cm * 360) / (wheelCircumfrence);
	}

	/**
	 * Convert cm of robot movement to encoder rotations
	 *
	 * @param cm
	 * @return
	 */
	public double encDistanceToCm(double encDistance)
	{
		return (encDistance / 360) * wheelCircumfrence;
	}

	/**
	 * Enum for how CmdMoveDistance determines when to end a move command.
	 *
	 * @author Jamie
	 *
	 */
	public enum MoveEndMode {
		BOTH, // ends when both sides have reached their targets.
		EITHER, // Stops both sides when either side has reached its target.
				// Force stops the move command of the slower side.
	}

	/**
	 * Command to move each side of the drivetrain a specified distance, using the
	 * MotionMagic control mode.
	 *
	 * Common logic shared by all of the autonomous movement commands
	 */
	public class CmdMotionMagicMove extends Command
	{
		final static double MOVEMENT_ERROR_THRESHOLD = 360 * Angle.DEGREES;
		final static double ERROR_PLATEAU_THRESHOLD = 0.01 * Angle.DEGREES;

		final static int ERROR_PLATEAU_COUNT = 25;

		protected double power;

		protected double leftAngle, rightAngle;

		protected MoveEndMode endMode;

		boolean leftDone, rightDone;
		int leftCount, rightCount;
		double lastLeftError, lastRightError;

		double leftError, rightError;

		boolean isInZone;

		boolean useScalars;

		/**
		 * @param endMode    - The MoveEndMode for this command.
		 * @param leftAngle  - Degrees to rotate the left wheel
		 * @param rightAngle - Degrees to rotate the right wheel
		 * @param power      - fractional power to move drive wheels at from 0 to 1
		 * @param useScalars - whether or not to scale output left and right powers
		 * @param timeoutMs  - The maximum time (in milliseconds) to run the command for
		 */
		public CmdMotionMagicMove(MoveEndMode endMode, double leftAngle, double rightAngle, double power, boolean useScalars, double timeoutMs) {
			super(timeoutMs / 1000.0);

			this.power = power;
			this.leftAngle = leftAngle;
			this.rightAngle = rightAngle;

			this.endMode = endMode;
			this.useScalars = useScalars;
		}

		protected void initialize() {
			Log.info("CmdMotionMagicMove", "Initializing...");

			clearEncoders();
			configureDriveMode(DriveMode.AUTONOMOUS);

			leftMotors.selectProfileSlot(0, 0);
			rightMotors.selectProfileSlot(0, 0);

			// Coast speed measured in nu/100ms
			double leftSpeed = (robotMaxSpeed * power * ((useScalars) ? leftSpeedScalar : 1.0));
			double rightSpeed = (robotMaxSpeed * power * ((useScalars) ? rightSpeedScalar : 1.0));

			if (leftAngle == 0) {
				leftSpeed = 0;
			} else if (rightAngle == 0) {
				rightSpeed = 0;
			} else if (Math.abs(leftAngle) > Math.abs(rightAngle)) {
				rightSpeed *= Math.abs(rightAngle / leftAngle);
			} else if (Math.abs(leftAngle) < Math.abs(rightAngle)) {
				leftSpeed *= Math.abs(leftAngle / rightAngle);
			}

			ControlMode leftMode = ControlMode.MotionMagic;
			ControlMode rightMode = ControlMode.MotionMagic;

			if (useScalars) {
				Log.info("CmdMotionMagicMove", "Using scalars.");
			}

			// motion magic does not work well when the distance is 0
			if (leftAngle == 0) {
				leftMode = ControlMode.Position;
			}
			if (rightAngle == 0) {
				rightMode = ControlMode.Position;
			}

			leftMotors.configMotionCruiseVelocity((int) leftSpeed, Constants.CAN_TIMEOUT);
			leftMotors.configMotionAcceleration((int) (leftSpeed), Constants.CAN_TIMEOUT);

			leftMotors.set(leftMode, leftAngle / Angle.CTRE_MAGENC_NU);

			rightMotors.configMotionCruiseVelocity((int) rightSpeed, Constants.CAN_TIMEOUT);
			rightMotors.configMotionAcceleration((int) (rightSpeed), Constants.CAN_TIMEOUT);

			rightMotors.set(rightMode, rightAngle / Angle.CTRE_MAGENC_NU);

			Log.debug("CmdMotionMagicMove",
				  "\n"
				+ "  Distances\n"
				+ "    L: " + leftAngle / Angle.CTRE_MAGENC_NU  + " nu\n"
				+ "    R: " + rightAngle / Angle.CTRE_MAGENC_NU + " nu\n"
				+ "  Speeds\n"
				+ "    L: " + leftSpeed  + " RPM"
				+ "    R: " + rightSpeed + " RPM"
			);

			try
			{
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		// Make this return true when this Command no longer needs to run
		// execute()
		protected boolean isFinished()
		{
			if (isTimedOut())
			{
				Log.unusual("CmdMotionMagicMove", "Autonomous Move Overtime.");
				return true;
			}

			leftError = leftMotors.getSelectedSensorPosition(0) * Angle.CTRE_MAGENC_NU - leftAngle;
			rightError = rightMotors.getSelectedSensorPosition(0) * Angle.CTRE_MAGENC_NU - rightAngle;

			Log.debug("CmdMotionMagicMove", "L=" + leftMotors.getSelectedSensorPosition(0) + "nu; err=" + leftError
					+ "deg, " + "R=" + rightMotors.getSelectedSensorPosition(0) + "nu; err=" + rightError + "deg.");

			if (leftError < -MOVEMENT_ERROR_THRESHOLD || rightError < -MOVEMENT_ERROR_THRESHOLD) {
				return false;
			}

			if (leftAngle == 0) {
				leftCount = ERROR_PLATEAU_COUNT + 1;
			}
			else if (Math.abs(leftError - lastLeftError) < ERROR_PLATEAU_THRESHOLD) {
				leftCount += 1;
			}
			else {
				leftCount = 0;
			}
			lastLeftError = leftError;
			leftDone = leftCount > ERROR_PLATEAU_COUNT;
			
			if (rightAngle == 0) {
				rightCount = ERROR_PLATEAU_COUNT + 1;
			}
			else if (Math.abs(rightError - lastRightError) < ERROR_PLATEAU_THRESHOLD) {
				rightCount += 1;
			}
			else {
				rightCount = 0;
			}
			lastRightError = rightError;
			rightDone = rightCount > ERROR_PLATEAU_COUNT;

			switch (endMode) {
			case BOTH:
				isInZone = leftDone && rightDone;
				break;
			case EITHER:
			default:
				isInZone = leftDone || rightDone;
				break;
			}

			return isInZone;
		}

		// Called once after isFinished returns true
		protected void end() {
			Log.info("CmdMotionMagicMove", "Ending normally.");
			Log.debug("CmdMotionMagicMove", "Final Errors:\n" + 
					  "  L=" + leftError  + "deg\n"
					+ "  R=" + rightError + "deg");

			stopMovement();
		}

		// Called when another command which requires one or more of the same
		// subsystems is scheduled to run
		protected void interrupted() {
			Log.info("CmdMotionMagicMove", "Interrupted.");

			stopMovement();
		}
	}

	/**
	 * Command to move forward the given amount of centimeters. Drives straight, if
	 * you've set up your speed multipliers properly.
	 */
	public class CmdDriveStraight extends CmdMotionMagicMove {
		/**
		 * @param distance  - The linear distance to drive forward
		 * @param power     - fractional power to move drive wheels at from 0 to 1
		 * @param timeoutMs - The maximum time to run the move (in milliseconds)
		 */
		public CmdDriveStraight(double distance, double power, int timeoutMs) {
			super(MoveEndMode.BOTH, cmToEncDegrees(distance), cmToEncDegrees(distance), power, false, timeoutMs);

		}
	}

	/**
	 * Command to turn in an arc with a certain radius for the specified amount of
	 * degrees.
	 *
	 * Unlike standard arc turn, it doesn't actually require omni wheels because the
	 * slippage is even less than an in place turn, so it's actually more accurate.
	 *
	 * Try to use this as frequently as you can, unless another method is quicker.
	 */
	public class CmdArcTurn extends CmdMotionMagicMove {
		/**
		 * @param radius    - The radius that the robot should drive the arc at.
		 * @param angle     - The distance to turn in degrees. Accepts negative values.
		 * @param dir       - The direction to turn towards.
		 * @param power     - The fractional power to drive the robot (from 0 to 1)
		 * @param timeoutMs - The maximum time to run the move (in milliseconds)
		 */
		public CmdArcTurn(double radius, double angle, Direction dir, double power, int timeoutMs)
		{
			super(MoveEndMode.BOTH, 0, 0, power, false, timeoutMs);

			// this formula is explained on the info repository wiki
			double innerAngularDist = cmToEncDegrees((angle * Math.PI / 180.0) * (radius - 0.5 * wheelBase));
			double outerAngularDist = cmToEncDegrees((angle * Math.PI / 180.0) * (radius + 0.5 * wheelBase));

			if (dir == Direction.RIGHT) {
				rightAngle = innerAngularDist;
				leftAngle = outerAngularDist;
			} else {
				rightAngle = outerAngularDist;
				leftAngle = innerAngularDist;
			}
		}
	}

	/**
	 * Command to to an arc turn in the specified amount of degrees.
	 *
	 * Sets the opposite motors from the direction provided, so turning LEFT would
	 * set the RIGHT motors.
	 */
	public class CmdInPlaceTurn extends CmdMotionMagicMove {
		/**
		 * @param angle     - The angle to turn in degrees. Accepts negative values.
		 * @param dir       - The direction to turn towards.
		 * @param power     - The relative speed to drive the turn at (from 0 to 1)
		 * @param timeoutMs - The maximum time to run the move (in milliseconds)
		 */
		public CmdInPlaceTurn(double angle, Direction dir, double power, int timeoutMs) {
			// the encoder counts are an in-depth calculation, so we don't set
			// them until after the super constructor
			super(MoveEndMode.BOTH, 0, 0, power, false, timeoutMs);

			// this formula is explained in the info repository wiki
			double wheelAngularDist = cmToEncDegrees((Math.PI * wheelBase) * (angle / 360.0));

			if (dir == Direction.RIGHT) {
				leftAngle = wheelAngularDist;
				rightAngle = -wheelAngularDist;
			} else {
				leftAngle = -wheelAngularDist;
				rightAngle = wheelAngularDist;
			}
		}
	}

	public abstract class CmdMotionProfileMove extends Command {
		public CmdMotionProfileMove(double timeoutMs) {
			super(timeoutMs / 1000.0);

			configureDriveMode(DriveMode.AUTONOMOUS);
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
		public void interrupted() {
			leftMotors.clearMotionProfileTrajectories();
			rightMotors.clearMotionProfileTrajectories();

			tankDrive(0, 0);
		}

		@Override
		protected void end() {
			leftMotors.clearMotionProfileTrajectories();
			rightMotors.clearMotionProfileTrajectories();

			tankDrive(0, 0);
		}
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
				leftMotors.processMotionProfileBuffer();
				rightMotors.processMotionProfileBuffer();
			});
		}

		@Override
		protected void initialize() {
			super.initialize();

			leftMotors.selectProfileSlot(0, 0);
			rightMotors.selectProfileSlot(0, 0);

			leftMotors.changeMotionControlFramePeriod((int) (Routemaker.durationMs / 2.3));
			rightMotors.changeMotionControlFramePeriod((int) (Routemaker.durationMs / 2.3));

			Routemaker rm = new Routemaker(power, waypoints);

			double speed;

			TrajectoryPoint trajPoint = new TrajectoryPoint();
			trajPoint.profileSlotSelect0 = 0;
			trajPoint.zeroPos = true;
			trajPoint.isLastPoint = false;

			ProfilePoint profilePoint;

			boolean first = true;

			do {
				speed = 1.0;
				if (rm.s < 0.2) {
					speed = (0.1 + rm.s) / 0.3;
				}
				else if (rm.s > 0.8) {
					speed = (1.1 - rm.s) / 0.7;
				}

				profilePoint = rm.getNextPoint(speed);
				// System.out.println(profilePoint.x + "," + profilePoint.y + " (" + profilePoint.durationMs + ")");
				System.out.println(profilePoint.leftDistance + "," + profilePoint.rightDistance);

				SmartDashboard.putNumber("Desired Left", profilePoint.leftDistance);
				SmartDashboard.putNumber("Desired Right", profilePoint.rightDistance);

				if (profilePoint.last)
					trajPoint.isLastPoint = true;

				trajPoint.timeDur = profilePoint.durationMs;

				trajPoint.position = profilePoint.leftDistance;
				trajPoint.velocity = profilePoint.leftSpeed;
				leftMotors.pushMotionProfileTrajectory(trajPoint);

				//System.out.println("left: " + trajPoint.position);

				trajPoint.position = profilePoint.rightDistance;
				trajPoint.velocity = profilePoint.rightSpeed;
				rightMotors.pushMotionProfileTrajectory(trajPoint);

				//System.out.println("right: " + trajPoint.position);
				//System.out.println("");


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

			if (super.isFinished()) {
				Log.info("CmdStaticRouteDrive", "Timed out.");
			}

			return super.isFinished() /* || leftStatus.isLast && rightStatus.isLast */;
		}

		@Override
		public void interrupted() {
			end();

			Log.info("CmdStaticRouteDrive", "Interrupted.");

		}

		@Override
		protected synchronized void end() {
			super.end();

			processNotifier.close();
			Log.info("CmdStaticRouteDrive", "Finished.");
		}
	}

	public class CmdDriveUntilStop extends Command {
		double power;

		public CmdDriveUntilStop(double power, int timeoutMs) {
			super(timeoutMs / 1000.0);

			this.power = power;
		}

		@Override
		protected void initialize() {
			arcadeDrive(-1 * power, 0, 1.0, false);

			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		@Override
		protected boolean isFinished() {
			return Math.abs(leftMotors.getSelectedSensorVelocity()) < 200 && Math.abs(rightMotors.getSelectedSensorVelocity()) < 200 || isTimedOut();
		}

		@Override
		protected void end() {
			leftMotors.set(ControlMode.PercentOutput, 0);
			rightMotors.set(ControlMode.PercentOutput, 0);

			if (isTimedOut()) {
				Log.unusual("CmdDriveUntilStopped", "Timed out.");
			}
		}

		@Override
		protected void interrupted() {
			end();
		}
	}

	/**
	 * Wrapper object to hold a single value--the calculated average wheelbase.
	 */
	public static class Wheelbase {
		public double wheelbase;
	}

	/**
	  * Calibration command to determine effective wheelbase of the robot. This is
	  * to account for the field material scrubbing against the wheels, resisting a
	  * turning motion.
	  *
	  * The effective wheelbase should always be larger than the measured wheelbase,
	  * but it will differ by a different factor for each robot. For instance, a
	  * 6-wheel tank drive with corner omnis will have a relatively small factor of
	  * difference, while a tank drive with pneumatic wheels will have a very large
	  * difference.
	  */
	public class CmdCalculateWheelbase extends Command {
		double leftPower, rightPower;

		double wheelbaseSum;
		double angularVelocity;

		Wheelbase wheelbase;

		double vL, vR;

		Gyro gyro;

		int timesRun;

		double previousTime;
		double previousAngle; 

		public CmdCalculateWheelbase(Wheelbase wheelbase, double leftPower, double rightPower, Gyro gyro, double durationMs) {
			super(durationMs / 1000.0);

			this.wheelbase = wheelbase;

			this.leftPower = leftPower;
			this.rightPower = rightPower;

			this.gyro = gyro;
		}

		@Override
		protected void initialize() {
			tankDrive(leftPower, rightPower);

			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		@Override
		protected void execute() {
			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

			vL = getLeftMotors().getSelectedSensorVelocity() * 10/4096 * wheelCircumfrence;
			vR = getRightMotors().getSelectedSensorVelocity() * 10/4096 * wheelCircumfrence;

			angularVelocity = gyro.getRate();

			wheelbaseSum += (vR - vL)/Math.toRadians(angularVelocity);
			timesRun += 1;
		}

		@Override
		protected boolean isFinished() {
			return isTimedOut();
		}

		@Override
		protected void end() {
			stopMovement();

			wheelbase.wheelbase = wheelbaseSum / timesRun;
			Log.info("CmdCalculateWheelBase", "Wheelbase: " + (wheelbase.wheelbase / Length.in) + " in");
		}
	}

	public static class FeedForwardPowerMultiplier {
		public double angularVelocity, ffpL, ffpR;

		public FeedForwardPowerMultiplier(double angularVelocity, double ffpL, double ffpR) {
			this.angularVelocity = angularVelocity;

			this.ffpL = ffpL;
			this.ffpR = ffpR;
		}
	}

	public static class FeedForwardPowerMultiplierSet {
		public int numLoops;
		private List<FeedForwardPowerMultiplier> ffpms = new ArrayList<FeedForwardPowerMultiplier>();
		private List<FeedForwardPowerMultiplier> ffpmsAvg = new ArrayList<FeedForwardPowerMultiplier>();

		public void addFeedForwardPowerMultiplier(double angularVelocity, double ffpL, double ffpR) {
			ffpms.add(new FeedForwardPowerMultiplier(angularVelocity, ffpL, ffpR));
		}
		public void addFeedForwardPowerMultiplierAvg(double angularVelocity, double ffpL, double ffpR) {
			ffpmsAvg.add(new FeedForwardPowerMultiplier(angularVelocity, ffpL, ffpR));
		}
		public void removeLastAverage(){
			ffpmsAvg.remove(ffpmsAvg.size() - 1);
		}
		public void removeLastAll(){
			for(int i = 0; i < numLoops; i++){
				ffpms.remove(ffpmsAvg.size() - 1);
			}
		}
		public String getAllCSV() {
			String csv = "angular velocity, ffpL, ffpR\n";

			for (FeedForwardPowerMultiplier ffpm : ffpms) {
				csv += ffpm.angularVelocity + ",";
				csv += ffpm.ffpL + ",";
				csv += ffpm.ffpR + "\n";
			}

			return csv;
		}
		public String getAvgCSV() {
			String csv = "angular velocity, ffpL, ffpR\n";

			for (FeedForwardPowerMultiplier ffpmAvg : ffpms) {
				csv += ffpmAvg.angularVelocity + ",";
				csv += ffpmAvg.ffpL + ",";
				csv += ffpmAvg.ffpR + "\n";
			}

			return csv;
		}
    }

	public class CmdGetFeedForwardPowerMultiplier extends Command {
		double leftPower, rightPower;

		double voltage;

		double angularVelocity, angularVelocitySum;
		double targetAngularVelocity;

		double vL, vR;
		double ffpLSum, ffpRSum;
		
		double ffpL, ffpR;

		Gyro gyro;

		int timesRun;
		int loopNum;

		FeedForwardPowerMultiplierSet feedForwardPowerMultiplierSet;

		public CmdGetFeedForwardPowerMultiplier(FeedForwardPowerMultiplierSet feedForwardPowerMultiplierSet, Gyro gyro, double leftPower, double rightPower, int durationMs) {
			super(durationMs / 1000.0);

			this.feedForwardPowerMultiplierSet = feedForwardPowerMultiplierSet;

			this.leftPower = leftPower;
			this.rightPower = rightPower;

			this.gyro = gyro;
		}

		@Override
		protected void initialize() {
			voltage = RobotController.getBatteryVoltage();
			tankDrive(leftPower, rightPower);

			try {
				Thread.sleep(1500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

			targetAngularVelocity = gyro.getRate();
			Log.info("targetW", String.valueOf(targetAngularVelocity));
		}

		@Override
		protected void execute() {
			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

			vL = getLeftMotors().getSelectedSensorVelocity() * 10/4096 * wheelCircumfrence;
			vR = getRightMotors().getSelectedSensorVelocity() * 10/4096 * wheelCircumfrence;

			angularVelocity = gyro.getRate();
			Log.info("currentAngularW", String.valueOf(angularVelocity));
			loopNum++;
			ffpL = leftPower/(vL * voltage/12.0);
			ffpR = rightPower/(vR * voltage/12.0);
			feedForwardPowerMultiplierSet.addFeedForwardPowerMultiplier(angularVelocity, ffpL, ffpR);
			if (RobotMath.isWithin(angularVelocity, targetAngularVelocity, 10.0)) {
				angularVelocitySum += angularVelocity;
				ffpLSum += ffpL;
				ffpRSum += ffpR;
				timesRun++;
			}
		}

		@Override
		protected boolean isFinished() {
			return isTimedOut();
		}

		@Override
		protected void end() {
			stopMovement();
			Log.info("angVelSum", String.valueOf(angularVelocitySum));
			Log.info("ffpLSum", String.valueOf(ffpLSum));
			Log.info("ffpRSum", String.valueOf(ffpRSum));
			feedForwardPowerMultiplierSet.addFeedForwardPowerMultiplierAvg(angularVelocitySum/timesRun, ffpLSum/timesRun, ffpRSum/timesRun);
			feedForwardPowerMultiplierSet.numLoops = timesRun;
			Log.info("# of loops", String.valueOf(loopNum));
			Log.info("# of valid loops", String.valueOf(timesRun))

;		}
 	}
}