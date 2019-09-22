package org.team3128.common.drive;

// import org.team3128.common.drive.routemaker.Routemaker;
// import org.team3128.common.drive.routemaker.ProfilePoint;
// import org.team3128.common.drive.routemaker.Waypoint;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.limelight.LimelightData;
import org.team3128.common.hardware.misc.TwoSpeedGearshift;
import org.team3128.common.hardware.gyroscope.Gyro;
import org.team3128.common.utility.Assert;
import org.team3128.common.utility.Constants;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.utility.enums.Direction;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.units.AngularSpeed;
import org.team3128.common.drive.base.ITankDrive;
import org.team3128.common.hardware.motor.LazyTalonSRX;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

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
	private LazyTalonSRX leftMotors, rightMotors;

	public LazyTalonSRX getLeftMotors() {
		return leftMotors;
	}

	public LazyTalonSRX getRightMotors() {
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
	// * Ratio between turns of the wheels to turns of the encoder
	// */
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

	private enum DriveMode {
		TELEOP(NeutralMode.Coast), AUTONOMOUS(NeutralMode.Brake);

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
	 * FPID constants for both left and righ drive sides in both Motion Magic/Motion
	 * Profile control mode as well as velocity control mode.
	 */
	public PIDConstants leftMotionProfilePID;
	// private PIDConstants leftVelocityPID, rightVelocityPID;

	public PIDConstants rightMotionProfilePID;

	// public double getGearRatio()
	// {
	// return gearRatio;
	// }

	// public void setGearRatio(double gearRatio)
	// {
	// this.gearRatio = gearRatio;
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
	 * @param leftMotors        The "lead" Talon SRX on the left side.
	 * @param rightMotors       The "lead" Talon SRX on the right side.
	 * @param wheelCircumfrence The amount of units of length the robot travels per
	 *                          rotation of the encoder stage
	 * @param gearRatio         The gear ratio of the turns of the wheels per turn
	 *                          of the encoder shaft
	 * @param wheelBase         The distance between the front and back wheel on a
	 *                          side
	 * @param track             distance across between left and right wheels
	 * @param robotMaxSpeed     The lesser of the maxiumum measured speeds for both
	 *                          drive sides, in native units per 100ms, of the robot
	 *                          driving on the ground at 100% throttle
	 */
	public static void initialize(LazyTalonSRX leftMotors, LazyTalonSRX rightMotors, double wheelCircumfrence,
			double wheelBase, int robotMaxSpeed) {
		instance = new SRXTankDrive(leftMotors, rightMotors, wheelCircumfrence, wheelBase, robotMaxSpeed);
	}

	private SRXTankDrive(LazyTalonSRX leftMotors, LazyTalonSRX rightMotors, double wheelCircumfrence, double wheelBase,
			int robotMaxSpeed) {
		this.leftMotors = leftMotors;
		this.rightMotors = rightMotors;

		this.wheelCircumfrence = wheelCircumfrence;
		this.wheelBase = wheelBase;
		// this.gearRatio = gearRatio;
		this.robotMaxSpeed = robotMaxSpeed;

		leftSpeedScalar = 1;
		rightSpeedScalar = 1;

		configureDriveMode(DriveMode.TELEOP);

		loadSRXPIDConstants();

		// if (gearRatio <= 0)
		// {
		// throw new IllegalArgumentException("Invalid gear ratio");
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
	public void arcadeDrive(double joyX, double joyY, double throttle, boolean fullSpeed) {
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

		spdL = leftSpeedScalar * RobotMath.clampPosNeg1(joyY - joyX);
		spdR = rightSpeedScalar * RobotMath.clampPosNeg1(joyY + joyX);

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

	/**
	 * Drive by providing motor powers for each side.
	 *
	 * @param powL the left side power.
	 * @param powR the right side power.
	 */
	public void tankDrive(double powL, double powR, boolean useScalars) {
		if (useScalars) {
			powL *= leftSpeedScalar;
			powR *= rightSpeedScalar;
		}

		tankDrive(powL, powR);
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

		// leftMotors.getSlotConfigs(configs, 1, Constants.CAN_TIMEOUT);
		// leftVelocityPID = new PIDConstants(configs.kF, configs.kP, configs.kI,
		// configs.kD);
		// Log.info("SRXTankDrive", "Left V: " + leftVelocityPID);

		rightMotors.getSlotConfigs(configs, 0, Constants.CAN_TIMEOUT);
		rightMotionProfilePID = new PIDConstants(configs.kF, configs.kP, configs.kI, configs.kD);
		Log.info("SRXTankDrive", "Right MP: " + rightMotionProfilePID);

		// rightMotors.getSlotConfigs(configs, 1, Constants.CAN_TIMEOUT);
		// rightVelocityPID = new PIDConstants(configs.kF, configs.kP, configs.kI,
		// configs.kD);
		// Log.info("SRXTankDrive", "Right V: " + rightVelocityPID);
	}

	public void setPID() {
		Log.info("SRXTankDrive", "Setting PID constants.");

		// leftMotors.config_kF(0, leftMotionProfilePID.kF);
		leftMotors.config_kP(0, leftMotionProfilePID.kP);
		leftMotors.config_kI(0, leftMotionProfilePID.kI);
		leftMotors.config_kD(0, leftMotionProfilePID.kD);

		// rightMotors.config_kF(0, rightMotionProfilePID.kF);
		rightMotors.config_kP(0, rightMotionProfilePID.kP);
		rightMotors.config_kI(0, rightMotionProfilePID.kI);
		rightMotors.config_kD(0, rightMotionProfilePID.kD);

		// leftMotors.config_kF(1, leftVelocityPID.kF);
		// leftMotors.config_kP(1, leftVelocityPID.kP);
		// leftMotors.config_kI(1, leftVelocityPID.kI);
		// leftMotors.config_kD(1, leftVelocityPID.kD);

		// rightMotors.config_kF(1, rightVelocityPID.kF);
		// rightMotors.config_kP(1, rightVelocityPID.kP);
		// rightMotors.config_kI(1, rightVelocityPID.kI);
		// rightMotors.config_kD(1, rightVelocityPID.kD);
	}

	/**
	 * Convert cm of robot movement to encoder movement in degrees
	 *
	 * @param cm
	 * @return
	 */
	public double cmToEncDegrees(double cm) {
		return (cm * 360) / (wheelCircumfrence);
	}

	/**
	 * Convert cm of robot movement to encoder rotations
	 *
	 * @param cm
	 * @return
	 */
	public double encDistanceToCm(double encDistance) {
		return (encDistance / 360) * wheelCircumfrence;
	}

	/**
	 * Enum for how CmdMoveDistance determines when to end a move command.
	 *
	 * @author Jamie
	 *
	 */
	public enum MoveEndMode {
		/**
		 * Ends the move command when both sides have reached their targets.
		 */
		BOTH,
		/**
		 * Stops both sides when either side has reached its target, force stopping the
		 * move command of the slower side.
		 */
		EITHER,
	}

	// double ccwLeftCutoff = 138;
	// double cwRightCutoff = 138;

	// double ccwRightCutoff = 170;
	// double cwLeftCutoff = 170;

	// double leftA = 6;
	// double leftB = 148;
	// double leftC = 0.291;

	// double leftFFCW = 0.3;
	// double leftFFCCW = 0.291;

	// double rightA = 6;
	// double rightB = 148;
	// double rightC = 0.28;

	// double rightFFCCW = 0.35;
	// double rightFFCW = 0.28;

	/**
	 * Command to move each side of the drivetrain a specified distance, using the
	 * MotionMagic control mode.
	 *
	 * Common logic shared by all of the autonomous movement commands
	 */
	public class CmdMotionMagicMove extends Command {
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
		public CmdMotionMagicMove(MoveEndMode endMode, double leftAngle, double rightAngle, double power,
				boolean useScalars, double timeoutMs) {
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

			// double angularVelocity = Math.toDegrees((rightSpeed - leftSpeed) /
			// wheelBase);

			// double ffLeft = 12.0 / RobotController.getBatteryVoltage() * leftA /
			// (angularVelocity + leftB) + leftC;
			// if (angularVelocity > ccwLeftCutoff) {
			// ffLeft = leftFFCCW;
			// } else if (angularVelocity < -cwLeftCutoff) {
			// ffLeft = leftFFCW;
			// }

			// double ffRight = 12.0 / RobotController.getBatteryVoltage() * rightA /
			// (angularVelocity + rightB) + rightC;
			// if (angularVelocity < -cwRightCutoff) {
			// ffRight = rightFFCW;
			// } else if (angularVelocity > ccwRightCutoff) {
			// ffRight = rightFFCCW;
			// }

			// leftMotors.config_kF(0, ffLeft);
			// leftMotors.config_kP(0, 0.05);

			// rightMotors.config_kF(0, ffRight);
			// rightMotors.config_kP(0, 0.05);

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
					"\n" + "  Distances\n" + "    L: " + leftAngle / Angle.CTRE_MAGENC_NU + " nu\n" + "    R: "
							+ rightAngle / Angle.CTRE_MAGENC_NU + " nu\n" + "  Speeds\n" + "    L: " + leftSpeed
							+ " RPM" + "    R: " + rightSpeed + " RPM");

			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		// Make this return true when this Command no longer needs to run
		// execute()
		protected boolean isFinished() {
			if (isTimedOut()) {
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
			} else if (Math.abs(leftError - lastLeftError) < ERROR_PLATEAU_THRESHOLD) {
				leftCount += 1;
			} else {
				leftCount = 0;
			}
			lastLeftError = leftError;
			leftDone = leftCount > ERROR_PLATEAU_COUNT;

			if (rightAngle == 0) {
				rightCount = ERROR_PLATEAU_COUNT + 1;
			} else if (Math.abs(rightError - lastRightError) < ERROR_PLATEAU_THRESHOLD) {
				rightCount += 1;
			} else {
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
			Log.debug("CmdMotionMagicMove",
					"Final Errors:\n" + "  L=" + leftError + "deg\n" + "  R=" + rightError + "deg");

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
		public CmdArcTurn(double radius, double angle, Direction dir, double power, int timeoutMs) {
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

	/*
	 * public class CmdStaticRouteDrive extends CmdMotionProfileMove { private
	 * MotionProfileStatus leftStatus, rightStatus; private Waypoint[] waypoints;
	 * private double power;
	 * 
	 * private Notifier processNotifier;
	 * 
	 * public CmdStaticRouteDrive(double power, double timeoutMs, Waypoint...
	 * waypoints) { super(timeoutMs);
	 * 
	 * this.power = power; this.waypoints = waypoints;
	 * 
	 * leftStatus = new MotionProfileStatus(); leftStatus.isLast = false;
	 * rightStatus = new MotionProfileStatus(); rightStatus.isLast = false;
	 * 
	 * processNotifier = new Notifier(() -> {
	 * leftMotors.processMotionProfileBuffer();
	 * rightMotors.processMotionProfileBuffer(); }); }
	 * 
	 * @Override protected void initialize() { super.initialize();
	 * 
	 * leftMotors.selectProfileSlot(0, 0); rightMotors.selectProfileSlot(0, 0);
	 * 
	 * leftMotors.changeMotionControlFramePeriod((int) (Routemaker.durationMs /
	 * 2.3)); rightMotors.changeMotionControlFramePeriod((int)
	 * (Routemaker.durationMs / 2.3));
	 * 
	 * Routemaker rm = new Routemaker(power, waypoints);
	 * 
	 * double speed;
	 * 
	 * TrajectoryPoint trajPoint = new TrajectoryPoint();
	 * trajPoint.profileSlotSelect0 = 0; trajPoint.zeroPos = true;
	 * trajPoint.isLastPoint = false;
	 * 
	 * ProfilePoint profilePoint;
	 * 
	 * boolean first = true;
	 * 
	 * do { speed = 1.0; if (rm.s < 0.2) { speed = (0.1 + rm.s) / 0.3; } else if
	 * (rm.s > 0.8) { speed = (1.1 - rm.s) / 0.7; }
	 * 
	 * profilePoint = rm.getNextPoint(speed); // System.out.println(profilePoint.x +
	 * "," + profilePoint.y + " (" + // profilePoint.durationMs + ")");
	 * System.out.println(profilePoint.leftDistance + "," +
	 * profilePoint.rightDistance);
	 * 
	 * SmartDashboard.putNumber("Desired Left", profilePoint.leftDistance);
	 * SmartDashboard.putNumber("Desired Right", profilePoint.rightDistance);
	 * 
	 * if (profilePoint.last) trajPoint.isLastPoint = true;
	 * 
	 * trajPoint.timeDur = profilePoint.durationMs;
	 * 
	 * trajPoint.position = profilePoint.leftDistance; trajPoint.velocity =
	 * profilePoint.leftSpeed; leftMotors.pushMotionProfileTrajectory(trajPoint);
	 * 
	 * // System.out.println("left: " + trajPoint.position);
	 * 
	 * trajPoint.position = profilePoint.rightDistance; trajPoint.velocity =
	 * profilePoint.rightSpeed; rightMotors.pushMotionProfileTrajectory(trajPoint);
	 * 
	 * // System.out.println("right: " + trajPoint.position); //
	 * System.out.println("");
	 * 
	 * if (first) { processNotifier.startPeriodic(Routemaker.durationSec / 2);
	 * 
	 * leftMotors.set(ControlMode.MotionProfile, 1);
	 * rightMotors.set(ControlMode.MotionProfile, 1);
	 * 
	 * first = false; }
	 * 
	 * } while (!profilePoint.last); }
	 * 
	 * @Override protected synchronized boolean isFinished() {
	 * leftMotors.getMotionProfileStatus(leftStatus);
	 * rightMotors.getMotionProfileStatus(rightStatus);
	 * 
	 * if (super.isFinished()) { Log.info("CmdStaticRouteDrive", "Timed out."); }
	 * 
	 * return super.isFinished(); // || leftStatus.isLast && rightStatus.isLast }
	 * 
	 * @Override public void interrupted() { end();
	 * 
	 * Log.info("CmdStaticRouteDrive", "Interrupted.");
	 * 
	 * }
	 * 
	 * @Override protected synchronized void end() { super.end();
	 * 
	 * processNotifier.close(); Log.info("CmdStaticRouteDrive", "Finished."); } }
	 */
	public class CmdDriveUntilStop extends Command {
		double power;
		double timeout;

		public CmdDriveUntilStop(double power, int timeoutMs) {
			super(timeoutMs / 1000.0);

			timeout = timeoutMs / 1000.0;

			this.power = power;
		}

		@Override
		protected void initialize() {
			tankDrive(power, power, true);

			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		@Override
		protected boolean isFinished() {
			if (timeSinceInitialized() < 0.5 * timeout)
				return false;

			// return Math.abs(leftMotors.getSelectedSensorVelocity()) < 200
			// && Math.abs(rightMotors.getSelectedSensorVelocity()) < 200 ||
			return isTimedOut();
		}

		@Override
		protected void end() {
			stopMovement();

			if (isTimedOut()) {
				Log.unusual("CmdDriveUntilStopped", "Timed out.");
			}
		}

		@Override
		protected void interrupted() {
			end();
		}
	}

	public class CmdPleaseWorkTurn extends Command {
		Gyro gyro;

		double angle;

		double radius;
		double power;

		PIDConstants innerPID, outerPID;

		private LazyTalonSRX outerMotors, innerMotors;

		private double vOuterCruise;
		private double wCruise;

		private double lastUpdateTime;

		// private double wCurrent;
		// private double vOuterCurrent;

		public CmdPleaseWorkTurn(Gyro gyro, double angle, double radius, Direction direction, double power,
				PIDConstants innerPID, PIDConstants outerPID, int timeoutMs) {
			super(timeoutMs / 1000.0);

			this.radius = ((direction == Direction.LEFT) ? 1 : -1) * radius;
			this.power = power;

			this.angle = angle;

			this.gyro = gyro;

			this.innerPID = innerPID;
			this.outerPID = outerPID;

			if (direction == Direction.LEFT) {
				outerMotors = rightMotors;
				innerMotors = leftMotors;
			} else {
				outerMotors = leftMotors;
				innerMotors = rightMotors;
			}
		}

		@Override
		protected void initialize() {
			vOuterCruise = power * robotMaxSpeed;

			wCruise = vOuterCruise * wheelCircumfrence * 10 / 4096 / radius;

			Log.info("Thing", "wCruise = " + wCruise);

			lastUpdateTime = RobotController.getFPGATime();
			gyro.setAngle(0);
		}

		@Override
		protected boolean isFinished() {
			outerMotors.set(ControlMode.PercentOutput, outerPID.kP * outerMotors.getSelectedSensorVelocity());
			innerMotors.set(ControlMode.PercentOutput, innerPID.kP * (wCruise - Math.toRadians(gyro.getRate())));

			lastUpdateTime = RobotController.getFPGATime();

			return gyro.getAngle() > angle || isTimedOut();
		}

		@Override
		protected void end() {
			stopMovement();
		}

	}

	public class CmdTargetAlignSimple extends Command {
		Gyro gyro;

		PIDConstants offsetPID;
		Limelight limelight;
		LimelightData data;

		private double feedForwardPower;

		private final double TARGET_HORIZONTAL_OFFSET = 0 * Angle.DEGREES;
		private double speedScalar;
		private double currentHorizontalOffset;
		private double currentError, previousError;
		private double currentTime, previousTime;
		private double distance;
		private double feedbackPower;

		double leftPower;
		double rightPower;
		double horizOffset;

		/**
		 * Constructor for the CmdTargetAlignSimple command.
		 * 
		 * @param gyro
		 * @param limelight
		 * @param initPower
		 * @param offsetPID
		 * @param timeoutMs
		 */
		public CmdTargetAlignSimple(Gyro gyro, Limelight limelight, double feedForwardPower, PIDConstants offsetPID,
				int timeoutMs) {
			super(timeoutMs / 1000.0);

			this.feedForwardPower = feedForwardPower;

			this.gyro = gyro;
			this.limelight = limelight;

			this.offsetPID = offsetPID;
		}

		@Override
		protected void initialize() {
			rightMotors.set(ControlMode.PercentOutput, feedForwardPower);
			leftMotors.set(ControlMode.PercentOutput, feedForwardPower);

			data = limelight.getValues(5);
			Log.info("CmdDynamicAdjust", String.valueOf(data.tx()));

			this.previousTime = RobotController.getFPGATime();
			this.previousError = data.tx();
			gyro.setAngle(0);
		}

		@Override
		protected boolean isFinished() {
			data = limelight.getValues(5);

			/**
			 * Only update the current error if there is a valid target, so if the target is
			 * lost, it just uses the last available value of the the error, which should
			 * theoretically force the robot to turn in the opposite direction until the
			 * target is back in the field of vision.
			 */

			distance = Math.abs(data.z());

			if (distance < 20 && data.tv() == 1) {
				Log.info("CmdDynamicAdjust", "Stop Initiated");
				rightMotors.set(ControlMode.PercentOutput, -0.1);
				leftMotors.set(ControlMode.PercentOutput, -0.1);
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				rightMotors.set(ControlMode.PercentOutput, 0);
				leftMotors.set(ControlMode.PercentOutput, 0);
			}
			if (data.tv() == 1) {
				currentHorizontalOffset = data.tx();

				currentTime = RobotController.getFPGATime() / 1000000.0;
				currentError = TARGET_HORIZONTAL_OFFSET - currentHorizontalOffset;

				/**
				 * PID feedback loop for the left and right powers based on the horizontal
				 * offset errors.
				 */
				feedbackPower = 0;

				feedbackPower += offsetPID.kP * currentError;
				feedbackPower += offsetPID.kD * (currentError - previousError) / (currentTime - previousTime);

				rightPower = feedForwardPower - feedbackPower;
				leftPower = feedForwardPower + feedbackPower;
				if (leftPower > rightPower) {
					speedScalar = 0.5 / leftPower;
					leftPower = leftPower * speedScalar;
					rightPower = rightPower * speedScalar;
				} else {
					speedScalar = 0.5 / rightPower;
					leftPower = leftPower * speedScalar;
					rightPower = rightPower * speedScalar;
				}
				// debug
				Log.info("CmdDynamicAdjust", "L: " + leftPower + "; R: " + rightPower);

				rightMotors.set(ControlMode.PercentOutput, rightPower);
				leftMotors.set(ControlMode.PercentOutput, leftPower);

				previousTime = currentTime;
				previousError = currentError;
			}

			// debug
			if (isTimedOut()) {
				Log.info("CmdTargetAlignSimple", "Timed out.");
			}
			if (data.tx() == 0) {
				Log.info("CmdTargetAlignSimple", "tx = 0");
			}

			return isTimedOut();
		}

		@Override
		protected void interrupted() {
			end();
		}

		@Override
		protected void end() {
			stopMovement();
		}

	}

	/**
	 * Wrapper object to hold all needed values to fit wheelbase to radius.
	 */
	public static class Wheelbase {
		public double wheelbase, radius, angularVelocity, linearVelocity, vL, vR;

		public Wheelbase(double wheelbase, double radius, double angularVelocity, double linearVelocity, double vL,
				double vR) {
			this.wheelbase = wheelbase;
			this.radius = radius;
			this.angularVelocity = angularVelocity;
			this.linearVelocity = linearVelocity;
			this.vL = vL;
			this.vR = vR;
		}
	}

	/**
	 * Wrapper for holding wheelbase set values and the average for each time the
	 * command is run. Mainly used for ease of printing data is csv format.
	 */
	public static class WheelbaseSet {
		private List<Wheelbase> wbSet = new ArrayList<Wheelbase>();
		public Wheelbase wbAvg;

		/**
		 * Add wheelbase to ongoing set of wheelbase/radius/angularVel/linearVel/vL/vR
		 * values.
		 * 
		 * @param wheelbase
		 * @param radius
		 * @param angularVelocity
		 * @param linearVelocity
		 * @param vL
		 * @param vR
		 */
		public void addWheelbase(double wheelbase, double radius, double angularVelocity, double linearVelocity,
				double vL, double vR) {
			wbSet.add(new Wheelbase(wheelbase, radius, angularVelocity, linearVelocity, vL, vR));
		}

		/**
		 * Populate a wheelbase object with the set of averaged values.
		 * 
		 * @param wheelbase
		 * @param radius
		 * @param angularVelocity
		 * @param linearVelocity
		 * @param vL
		 * @param vR
		 */
		public void setAverage(double wheelbase, double radius, double angularVelocity, double linearVelocity,
				double vL, double vR) {
			wbAvg = new Wheelbase(wheelbase, radius, angularVelocity, linearVelocity, vL, vR);
		}

		/**
		 * @return String in csv format populated with the entire dataset of
		 *         wheelbase/radius/angularVel/linearVel/vL/vR values.
		 */
		public String getAllCSV() {
			String csv = "";

			for (Wheelbase wb : wbSet) {
				csv += wb.wheelbase + ",";
				csv += wb.radius + ",";
				csv += wb.angularVelocity + ",";
				csv += wb.linearVelocity + ",";
				csv += wb.vL + ",";
				csv += wb.vR + "\n";
			}

			return csv;
		}

		/**
		 * @return String in csv format populated with the wheelbase object of
		 *         averaged(& within range) wheelbase/radius/angularVel/linearVel/vL/vR
		 *         values.
		 */
		public String getAvgCSV() {
			String csv = "";

			csv += wbAvg.wheelbase + ",";
			csv += wbAvg.radius + ",";
			csv += wbAvg.angularVelocity + ",";
			csv += wbAvg.linearVelocity + ",";
			csv += wbAvg.vL + ",";
			csv += wbAvg.vR + "\n";

			return csv;
		}
	}

	/**
	 * Calibration command to determine effective wheelbase of the robot. This is to
	 * account for the field material scrubbing against the wheels, resisting a
	 * turning motion.
	 *
	 * The effective wheelbase should always be larger than the measured wheelbase,
	 * but it will differ by a different factor for each robot. For instance, a
	 * 6-wheel tank drive with corner omnis will have a relatively small factor of
	 * difference, while a tank drive with pneumatic wheels will have a very large
	 * difference.
	 */
	public class CmdCalculateWheelbase extends Command {
		private final double VELOCITY_PLATEAU_RANGE = 25;// 2;
		double leftPower, rightPower;

		double wheelbase, radius, linearVelocity, angularVelocity, vL, vR;
		double wheelbaseSum, radiusSum, linearVelocitySum, angularVelocitySum, vLSum, vRSum;

		WheelbaseSet wheelbaseSet;
		Gyro gyro;

		int executeCount;
		int inRangeCount;

		double targetRadius;

		/**
		 * @param wheelbaseSet
		 * @param leftPower
		 * @param rightPower
		 * @param gyro
		 * @param durationMs
		 */
		public CmdCalculateWheelbase(WheelbaseSet wheelbaseSet, double leftPower, double rightPower, Gyro gyro,
				double durationMs) {
			super(durationMs / 1000.0);

			this.wheelbaseSet = wheelbaseSet;

			this.leftPower = leftPower;
			this.rightPower = rightPower;

			this.gyro = gyro;
		}

		@Override
		protected void initialize() {
			// might need to account for voltage:
			// voltage = RobotController.getBatteryVoltage();
			tankDrive(leftPower, rightPower);

			double currentRadius = 0;
			double previousRadius = 0;
			int plateauCount = 0;

			while (true) {
				vL = getLeftMotors().getSelectedSensorVelocity() * 10 / 4096 * wheelCircumfrence;
				vR = getRightMotors().getSelectedSensorVelocity() * 10 / 4096 * wheelCircumfrence;
				linearVelocity = (vL + vR) / 2;
				angularVelocity = Math.abs(Math.toRadians(gyro.getRate()));
				currentRadius = linearVelocity / angularVelocity;

				Log.info("CmdCalculateWheelbase", "Plateau Radius: " + currentRadius);

				if (Math.abs(currentRadius - previousRadius) < VELOCITY_PLATEAU_RANGE) {
					plateauCount += 1;
				} else {
					plateauCount = 0;
				}

				if (plateauCount > 10)
					break;
				previousRadius = currentRadius;

				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
			radiusSum = 0;
			int radiusCount = 0;

			for (int i = 0; i < 25; i++) {
				vL = getLeftMotors().getSelectedSensorVelocity() * 10 / 4096 * wheelCircumfrence;
				vR = getRightMotors().getSelectedSensorVelocity() * 10 / 4096 * wheelCircumfrence;

				linearVelocity = (vL + vR) / 2;
				angularVelocity = Math.abs(Math.toRadians(gyro.getRate()));

				currentRadius = linearVelocity / angularVelocity;

				radiusSum += currentRadius;
				radiusCount += 1;
			}

			targetRadius = radiusSum / radiusCount;
			radiusSum = 0;

		}

		@Override
		protected void execute() {
			// radiusSum = 0;
			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			executeCount++;

			vL = getLeftMotors().getSelectedSensorVelocity() * 10 / 4096 * wheelCircumfrence;
			vR = getRightMotors().getSelectedSensorVelocity() * 10 / 4096 * wheelCircumfrence;

			linearVelocity = (vL + vR) / 2;
			angularVelocity = Math.abs(Math.toRadians(gyro.getRate()));

			radius = linearVelocity / angularVelocity;
			wheelbase = Math.abs(vR - vL) / angularVelocity;

			Log.info("CmdCalculateWheelbase", "Loop radius = " + radius);

			if (RobotMath.isWithin(radius, targetRadius, 25.0)) {
				wheelbaseSet.addWheelbase(wheelbase, radius, angularVelocity, linearVelocity, vL, vR);
			}

			if (RobotMath.isWithin(radius, targetRadius, 10.0)) {
				wheelbaseSum += wheelbase;
				radiusSum += radius;
				angularVelocitySum += angularVelocity;
				linearVelocitySum += linearVelocity;
				vLSum += vL;
				vRSum += vR;

				inRangeCount++;
			}
		}

		@Override
		protected boolean isFinished() {
			return isTimedOut();
		}

		@Override
		protected void end() {
			stopMovement();
			double avgWheelbase = wheelbaseSum / inRangeCount;
			double avgRadius = radiusSum / inRangeCount;
			double avgAngularVelocity = angularVelocitySum / inRangeCount;
			double avgLinearVelocity = linearVelocity / inRangeCount;
			double avgVL = vLSum / inRangeCount;
			double avgVR = vRSum / inRangeCount;

			Log.info("CmdCalculateWheelbase",
					"Completed...\n" + "\tTarget Radius: " + targetRadius + "\n" + "\tRuntime Information:\n"
							+ "\t\tTotal Loops: " + executeCount + "\n" + "\t\tIn-Range Loops: " + inRangeCount + "\n"
							+ "\tFinal Average Values\n" + "\t\tRadius: " + avgRadius + "\n" + "\t\tWheelbase: "
							+ avgWheelbase + "\n" + "\t\tAngular Velocity: " + avgAngularVelocity + "\n"
							+ "\t\tLinear Velocity: " + avgLinearVelocity + "\n" + "\t\tLeft Velocity: " + avgVL + "\n"
							+ "\t\tRight Velocity: " + avgVR);
			wheelbaseSet.setAverage(avgWheelbase, avgRadius, avgAngularVelocity, avgLinearVelocity, avgVL, avgVR);
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
		private List<FeedForwardPowerMultiplier> ffpms = new ArrayList<FeedForwardPowerMultiplier>();
		public FeedForwardPowerMultiplier ffpmAvg;

		public void addFeedForwardPowerMultiplier(double angularVelocity, double ffpL, double ffpR) {
			ffpms.add(new FeedForwardPowerMultiplier(angularVelocity, ffpL, ffpR));
		}

		public void setAverage(double angularVelocity, double ffpL, double ffpR) {
			ffpmAvg = new FeedForwardPowerMultiplier(angularVelocity, ffpL, ffpR);
		}

		public String getAllCSV() {
			String csv = "";

			for (FeedForwardPowerMultiplier ffpm : ffpms) {
				csv += ffpm.angularVelocity + ",";
				csv += ffpm.ffpL + ",";
				csv += ffpm.ffpR + "\n";
			}

			return csv;
		}

		public String getAvgCSV() {
			String csv = "";

			csv += ffpmAvg.angularVelocity + ",";
			csv += ffpmAvg.ffpL + ",";
			csv += ffpmAvg.ffpR + "\n";

			return csv;
		}
	}

	public class CmdGetFeedForwardPowerMultiplier extends Command {
		private final double VELOCITY_PLATEAU_RANGE = 2;

		double leftPower, rightPower;

		double voltage;

		double angularVelocity, angularVelocitySum;
		double targetAngularVelocity;

		double vL, vR;
		double ffpmLSum, ffpmRSum;

		double ffpmL, ffpmR;

		Gyro gyro;

		int inRangeCount;
		int executeCount;

		FeedForwardPowerMultiplierSet feedForwardPowerMultiplierSet;

		public CmdGetFeedForwardPowerMultiplier(FeedForwardPowerMultiplierSet feedForwardPowerMultiplierSet, Gyro gyro,
				double leftPower, double rightPower, int durationMs) {
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

			double currentAngularVelocity = 0;
			double previousAngularVelocity = 0;
			int plateauCount = 0;

			while (true) {
				currentAngularVelocity = gyro.getRate();
				Log.info("CmdGetFeedForwardPowerMultiplier", "Plateau Velocity: " + currentAngularVelocity);

				if (Math.abs(gyro.getRate() - previousAngularVelocity) < VELOCITY_PLATEAU_RANGE) {
					plateauCount += 1;
				} else {
					plateauCount = 0;
				}

				if (plateauCount > 10)
					break;
				previousAngularVelocity = currentAngularVelocity;

				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}

			double angularVelocitySum = 0;
			int angularVelocityCount = 0;

			double rate;

			for (int i = 0; i < 25; i++) {
				rate = gyro.getRate();

				angularVelocitySum += rate;
				angularVelocityCount += 1;

				// Log.info("CmdGetFeedForwardPowerMultiplier", "" + rate);
			}

			targetAngularVelocity = angularVelocitySum / angularVelocityCount;
		}

		@Override
		protected void execute() {
			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

			executeCount += 1;

			vL = getLeftMotors().getSelectedSensorVelocity();
			vR = getRightMotors().getSelectedSensorVelocity();

			angularVelocity = gyro.getRate();
			Log.info("CmdGetFeedForwardPowerMultiplier", "Loop omega = " + angularVelocity);

			ffpmL = leftPower / (vL * voltage / 12.0);
			ffpmR = rightPower / (vR * voltage / 12.0);

			if (RobotMath.isWithin(angularVelocity, targetAngularVelocity, 35.0)) {
				feedForwardPowerMultiplierSet.addFeedForwardPowerMultiplier(angularVelocity, ffpmL, ffpmR);
			}

			if (RobotMath.isWithin(angularVelocity, targetAngularVelocity, 10.0)) {
				angularVelocitySum += angularVelocity;

				ffpmLSum += ffpmL;
				ffpmRSum += ffpmR;

				inRangeCount += 1;
			}
		}

		@Override
		protected boolean isFinished() {
			return isTimedOut();
		}

		@Override
		protected void end() {
			stopMovement();

			double avgAngularVelocity = angularVelocitySum / inRangeCount;
			double ffpmLAvg = ffpmLSum / inRangeCount;
			double ffpmRAvg = ffpmRSum / inRangeCount;

			Log.info("CmdGetFeedForwardPowerMultiplier",
					"Completed...\n" + "\tTarget Angular Velocity: " + targetAngularVelocity + "\n"
							+ "\tRuntime Information:\n" + "\t\tTotal Loops: " + executeCount + "\n"
							+ "\t\tIn-Range Loops: " + inRangeCount + "\n" + "\tFinal Average Values\n"
							+ "\t\tAngular Velocity: " + avgAngularVelocity + "\n" + "\t\tLeft FFPM: " + ffpmLAvg + "\n"
							+ "\t\tRight FFPM: " + ffpmRAvg);

			feedForwardPowerMultiplierSet.setAverage(angularVelocitySum / inRangeCount, ffpmLSum / inRangeCount,
					ffpmRSum / inRangeCount);
		}
	}
}