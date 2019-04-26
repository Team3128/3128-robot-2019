package org.team3128.gromit.mechanisms;

import org.team3128.common.util.Constants;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.units.Angle;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;

public class FourBar
{
	/**
	 * The amount of native units read by the encoder equal to 1 degree.
	 */
	public double ANGLE_ENCODER_RATIO;
	public double error;

	private final double ALLOWABLE_ERROR = 2 * Angle.DEGREES;

    public enum FourBarAngleTarget {
		VERTICAL(90 * Angle.DEGREES),

		CARGO_INTAKE(-20 * Angle.DEGREES),

		CARGO_SHIP(80 * Angle.DEGREES),

		CARGO_LOW(80 * Angle.DEGREES), 
		CARGO_MID(80 * Angle.DEGREES),
		CARGO_HIGH(82 * Angle.DEGREES),
		
		HATCH_LOW(-60 * Angle.DEGREES),
		HATCH_HIGH(54 * Angle.DEGREES);

		public double targetAngle;

        private FourBarAngleTarget(double angle) {
            this.targetAngle = angle;
		}
	}
	
	public enum FourBarControlMode {
		PERCENT(1, "Percent Output"),
		POSITION(1, "Position"),
		ZEROING(1, "Zeroing");

		private int pidSlot;
		private String name;

		private FourBarControlMode(int pidSlot, String name)
		{
			this.pidSlot = pidSlot;
			this.name = name;
		}

		public int getPIDSlot()
		{
			return pidSlot;
		}

		public String getName()
		{
			return name;
		}

	}

	public void setControlMode(FourBarControlMode controlMode) {
		if (this.controlMode != controlMode) {
			Log.debug("FourBar", "Setting control mode to " + controlMode.name() + " with Talon SRX onboard PID slot " + controlMode.getPIDSlot());

			this.controlMode = controlMode;
			fourBarMotor.selectProfileSlot(controlMode.getPIDSlot(), 0);
		}
	}

	private TalonSRX fourBarMotor;
	
	private DigitalInput limitSwitch;

	public double limitSwitchAngle;
	public int maxVelocity;

	// Control Notifier
	private FourBarControlMode controlMode = FourBarControlMode.PERCENT;
	private Notifier controlNotifier;

	public final double PEAK_BRAKE_POWER = 0.15;
	public final double BRAKE_TRIG_FUDGE = 0.095;

	public final double ZEROING_POWER = 0.8272;

	public double maxAngle = +95.0 * Angle.DEGREES;
	public double minAngle = -90.0 * Angle.DEGREES;

	public boolean disabled = true;

	public boolean canLower, canRaise;

	/**
	 * Target variable set OUTSIDE of the control thread, i.e. a joystick power or an angle.
	 */
	private double desiredTarget = 0;

	/**
	 * Target variable set WITHIN the logic of the control thread
	 */
	private double currentTarget, previousTarget;

	private double lastTime, lastError;

	private final double JOYSTICK_THRESHOLD = 0.1;

	public boolean override = false;

	private int zeroVelocityCount = 0;

	private double upKP = 0.23, downKP = 0.01, kD = 0;

	
	private static FourBar instance = null;
	public static FourBar getInstance() {
		if (instance != null) {
			return instance;
		}

		Log.fatal("FourBar", "Attempted to get instance before initializtion! Call initialize(...) first.");
		return null;
	}

	public static void initialize(TalonSRX fourBarMotor, DigitalInput limitSwitch, double ratio, double limitSwitchAngle, int maxVelocity) {
		instance = new FourBar(fourBarMotor, limitSwitch, ratio, limitSwitchAngle, maxVelocity);
	}

	private FourBar(TalonSRX fourBarMotor, DigitalInput limitSwitch, double ratio, double limitSwitchAngle, int maxVelocity) {
		this.fourBarMotor = fourBarMotor;

		this.limitSwitch = limitSwitch;

		this.ANGLE_ENCODER_RATIO = ratio;

		this.limitSwitchAngle = limitSwitchAngle;
		this.maxVelocity = maxVelocity;

		controlNotifier = new Notifier(this::controlLoop);
		controlNotifier.startPeriodic(0.010);
	}

	private void controlLoop() {
		if (getLimitSwitch()) {
			setCurrentAngle(limitSwitchAngle);
		}

		currentTarget = 0;

		if (!disabled) {
			switch (controlMode) {
				case PERCENT:
					canRaise = getCurrentAngle() < maxAngle - 2 * Angle.DEGREES;
					canLower = getCurrentAngle() > minAngle + 1 * Angle.DEGREES;
					
					if (override) {
						currentTarget = desiredTarget;
					}
					else {
						if (desiredTarget > 0 && canRaise) {
							currentTarget = 0.7 * getAdjustedTarget(desiredTarget);
						}
						else if (desiredTarget < 0 && canLower) {
							currentTarget = 0.4 * getAdjustedTarget(desiredTarget);
						}
						
						if (Math.abs(currentTarget) < 0.0001 && canRaise && canLower) {
							brake();
						}
					}

					break;

				case POSITION:
					lastError = error;
					error = desiredTarget - getCurrentAngle();
					
					currentTarget = getFeedForwardPower() + (error > 0 ? upKP : downKP) * error + kD * (error - lastError) * 1000000 / (RobotController.getFPGATime() - lastTime);
					lastTime = RobotController.getFPGATime();

					break;

				case ZEROING:
					currentTarget = ZEROING_POWER;

					if (Math.abs(fourBarMotor.getSelectedSensorVelocity()) < 2) {
						zeroVelocityCount += 1;
					}
					else {
						zeroVelocityCount = 0;
					}
					
					if (zeroVelocityCount > 30 || getLimitSwitch()) {
						Log.info("FourBar", "Zeroing sequence hit " + (zeroVelocityCount > 30 ? "SOFT" : "HARD") + " stop. Braking now...");
						
						angleControl(85.0 * Angle.DEGREES);
						
						zeroVelocityCount = 0;
					}

					break;
			}
		}
			
		if (Math.abs(currentTarget - previousTarget) > 0.0001) {
			fourBarMotor.set(ControlMode.PercentOutput, currentTarget);
			
			previousTarget = currentTarget;
		}
	}

	private double getAdjustedTarget(double joystick) {
		if (Math.abs(joystick) < JOYSTICK_THRESHOLD) return 0;

		return (joystick > 0) ? 1 : -1 * Math.pow((Math.abs(joystick) - JOYSTICK_THRESHOLD) / (1 - JOYSTICK_THRESHOLD), 2);
	}

	public boolean getLimitSwitch() {
		return !limitSwitch.get();
	}

	public double getCurrentAngle() {
		return fourBarMotor.getSelectedSensorPosition(0) / ANGLE_ENCODER_RATIO;
	}

	public void setCurrentAngle(double angle) {
		//Log.info("FourBar", "Setting current angle to " + angle + " degrees.");
		fourBarMotor.setSelectedSensorPosition((int) (angle * ANGLE_ENCODER_RATIO), 0, Constants.CAN_TIMEOUT);
	}

    public void setTarget(FourBarAngleTarget angleTarget)
	{
		Log.info("FourBar", "Going to " + angleTarget.targetAngle + " degrees.");
		angleControl(angleTarget.targetAngle);
	}
	
	private double getFeedForwardPower() {
		return PEAK_BRAKE_POWER * (BRAKE_TRIG_FUDGE + (1 - BRAKE_TRIG_FUDGE) * RobotMath.cos(getCurrentAngle()));
	}
    
	public void powerControl(double joystick) {
		setControlMode(FourBarControlMode.PERCENT);
		
		desiredTarget = joystick;
	}

	public void brake() {
		angleControl(getCurrentAngle());
	}

	public void zero() {
		setControlMode(FourBarControlMode.ZEROING);
	}

	public void angleControl(double angle) {
		setControlMode(FourBarControlMode.POSITION);

		desiredTarget = angle;

		lastTime = RobotController.getFPGATime();
		lastError = desiredTarget - getCurrentAngle();
	}

	public class CmdZero extends Command {
		public CmdZero() {
			super(0.25);
		}

		@Override
		protected void initialize() {
			zero();
		}

		@Override
		protected boolean isFinished()
		{
			return controlMode != FourBarControlMode.ZEROING || isTimedOut();
		}

		@Override
		protected void end() {
			Log.info("CmdZero", "Four Bar zeroing complete.");
		}

		@Override
		protected void interrupted() {
			Log.info("CmdZero", "Four Bar zeroing interrupted.");
		}
	}

	public class CmdAngleControl extends Command {
		private FourBarAngleTarget angleTarget;
		private double error;

		public CmdAngleControl(FourBarAngleTarget angleTarget) {
			super(2);
			this.angleTarget = angleTarget;
		}

		@Override
		protected void initialize() {
			setTarget(angleTarget);
			Log.debug("CmdAngleControl", "Setting target to " + angleTarget.name() + " with angle of " + angleTarget.targetAngle + " degrees.");
		}

		@Override
		protected void end() {
			powerControl(0);
			Log.debug("CmdAngleControl", "Command Finished. Final angle = " + getCurrentAngle() + " degrees.");
		}

		@Override
		protected void interrupted() {
			Log.debug("CmdAngleControl", "Command Interrupted.");
			end();
		}

		@Override
		protected boolean isFinished() {
			error = getCurrentAngle() - angleTarget.targetAngle;
			Log.debug("CmdAngleControl", "Error: " + error + "deg");

			return Math.abs(error) < ALLOWABLE_ERROR || isTimedOut();
		}
	}

}