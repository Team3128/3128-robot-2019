package org.team3128.gromit.mechanisms;

import org.team3128.common.util.Constants;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.units.Angle;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;

public class FourBar
{
	/**
	 * The angle in native units equal to 1 degree.
	 */
    public double ratio, error;

	private final double allowableError = 2 * Angle.DEGREES;

    public enum FourBarState {
		VERTICAL(90*Angle.DEGREES), //DEBUG
		SHIP_AND_LOADING(-55 * Angle.DEGREES),
		// HATCH_DROP_SHIP_LOADING(-53 * Angle.DEGREES),

		CARGO_LOW(80 * Angle.DEGREES), 
		//64
		CARGO_MID(80 * Angle.DEGREES),
		CARGO_LOADING_STATION(55 * Angle.DEGREES),
		// HATCH_DROP_ROCKET_LOW(-65 * Angle.DEGREES),
		
		CARGO_INTAKE(-22 * Angle.DEGREES),
		//17
		//HATCH_LOW(15 * Angle.DEGREES),
		HATCH_LOW(-60 * Angle.DEGREES),
		//64
		HATCH_HIGH(54 * Angle.DEGREES),
		CARGO_HIGH(82 * Angle.DEGREES);
		//68
		public double targetAngle;

        private FourBarState(double angle) {
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

	public void setControlMode(FourBarControlMode mode) {
		if (mode != controlMode)
		{
			controlMode = mode;
			Log.debug("FourBar", "Setting control mode to " + mode.name() + ", with PID slot " + mode.getPIDSlot());
			fourBarMotor.selectProfileSlot(mode.getPIDSlot(), 0);
		}
	}

	TalonSRX fourBarMotor;
	FourBarControlMode controlMode;
	FourBarState state;
	
	private Thread fourBarThread;
	public DigitalInput limitSwitch;

	double limitSwitchAngle;
	int maxVelocity;

	// Control Thread Variables
	public double PEAK_BRAKE_POWER = 0.15;
	public double BRAKE_TRIG_FUDGE = 0.095;

	public double ZEROING_VELOCITY = 30 * Angle.DEGREES * ratio / 10.0;

	public double maxAngle = +95.0 * Angle.DEGREES;
	public double minAngle = -90.0 * Angle.DEGREES;

	public boolean disabled = false;

	public boolean canLower = false;
	public boolean canRaise = true;

	private double desiredTarget = 0;

	private double lastTime;
	private double lastError;

	private double joystickThreshold = 0.1;

	public boolean override = false;

	
	private static FourBar instance = null;
	public static FourBar getInstance() {
		if (instance != null) {
			return instance;
		}

		Log.fatal("FourBar", "Attempted to get instance before initializtion! Call initialize(...) first.");
		return null;
	}

	public static void initialize(TalonSRX fourBarMotor, FourBarState state, DigitalInput limitSwitch, double ratio, double limitSwitchAngle, int maxVelocity) {
		instance = new FourBar(fourBarMotor, state, limitSwitch, ratio, limitSwitchAngle, maxVelocity);
	}

	private FourBar(TalonSRX fourBarMotor, FourBarState state, DigitalInput limitSwitch, double ratio, double limitSwitchAngle, int maxVelocity) {
		this.fourBarMotor = fourBarMotor;
		this.state = state;

		this.limitSwitch = limitSwitch;

		this.ratio = ratio;

		this.limitSwitchAngle = limitSwitchAngle;
		this.maxVelocity = maxVelocity;

		powerControl(0);
				
		fourBarThread = new Thread(() ->
		{
			int zeroVelocityCount = 0;

			double currentTarget = 0;
			double previousTarget = 0;

			double kP;
			double kD;

			while (true)
			{
				// Zeroing Logic
				if (this.controlMode == FourBarControlMode.ZEROING) {
					if (Math.abs(fourBarMotor.getSelectedSensorVelocity()) < 5) {
						zeroVelocityCount += 1;
					}
					else {
						zeroVelocityCount = 0;
					}

					if (zeroVelocityCount > 30 || this.getLimitSwitch() /** || maybe cleverly implement current limiting*/) {
						this.state = FourBarState.VERTICAL;
						Log.info("FourBar", "Zeroing sequence hit hard/soft stop. Braking now...");

						this.angleControl(85.0 * Angle.DEGREES);

						zeroVelocityCount = 0;
					}
				}

				// Limit switch setting
				if (this.getLimitSwitch()) {
					this.setCurrentAngle(this.limitSwitchAngle);

					this.state = FourBarState.VERTICAL;
				}

				// Control loop logic
				if (this.disabled) {
					this.fourBarMotor.set(ControlMode.PercentOutput, 0);
				}
				else {
					currentTarget = 0;

					if (this.controlMode == FourBarControlMode.PERCENT) {
						if (this.override) {
							currentTarget = desiredTarget;
							this.fourBarMotor.set(ControlMode.PercentOutput, currentTarget);
						}
						else {
							this.canRaise = this.getCurrentAngle() < this.maxAngle - 2 * Angle.DEGREES;
							this.canLower = this.getCurrentAngle() > this.minAngle + 1 * Angle.DEGREES;

							if (desiredTarget > 0 && this.canRaise) {
								currentTarget = 0.7 * getAdjustedTarget(desiredTarget);
							}
							else if (desiredTarget < 0 && this.canLower) {
								currentTarget = 0.4 * getAdjustedTarget(desiredTarget);
							}

							if ((Math.abs(currentTarget) < 0.0001 && this.canRaise && this.canLower)) {
								this.brake();
							}
						}
					}
					else if (this.controlMode == FourBarControlMode.POSITION) {
						lastError = this.error;
						this.error = desiredTarget - this.getCurrentAngle();

						if (this.error > 0) {
							kP = 0.23;
						}
						else {
							kP = 0.01;
						}
						kD = 0;

						currentTarget = this.getFeedForwardPower() + kP * this.error + kD * (this.error - lastError) * 1000000 / (RobotController.getFPGATime() - this.lastTime);
						this.lastTime = RobotController.getFPGATime();
					}
					else if (this.controlMode == FourBarControlMode.ZEROING) {
						lastError = this.error;
						this.error = desiredTarget - fourBarMotor.getSelectedSensorVelocity();

						kP = 0.3;
						kD = 0;

						currentTarget = 0.7 /** + kP * this.error + kD * (this.error - lastError) * 1000000 / (RobotController.getFPGATime() - this.lastTime) */;
						this.lastTime = RobotController.getFPGATime();
					}

					if (Math.abs(currentTarget - previousTarget) > 0.0001) {
						this.fourBarMotor.set(ControlMode.PercentOutput, currentTarget);

						previousTarget = currentTarget;
					}
				}

				try
				{
					Thread.sleep(10);
				}
				catch (InterruptedException e)
				{
					e.printStackTrace();
				}
			}

		});

		fourBarThread.start();
	}

	private double getAdjustedTarget(double joystick) {
		if (Math.abs(joystick) < joystickThreshold) return 0;

		return (joystick > 0) ? 1 : -1 * Math.pow((Math.abs(joystick) - joystickThreshold) / (1 - joystickThreshold), 2);
	}

	public double getCurrentAngle() {
		return fourBarMotor.getSelectedSensorPosition(0) / ratio;
	}

	public void setCurrentAngle(double angle) {
		//Log.info("FourBar", "Setting current angle to " + angle + " degrees.");
		fourBarMotor.setSelectedSensorPosition((int) (angle * ratio), 0, Constants.CAN_TIMEOUT);
	}

    public void setState(FourBarState fourBarState)
	{
		state = fourBarState;

		Log.info("FourBar", "Going to " + state.targetAngle + " degrees.");
		angleControl(state.targetAngle);
	}
	
	private double getFeedForwardPower() {
		return PEAK_BRAKE_POWER * (BRAKE_TRIG_FUDGE + (1 - BRAKE_TRIG_FUDGE)*RobotMath.cos(getCurrentAngle()));
	}
    
	public void powerControl(double joystick)
	{
		setControlMode(FourBarControlMode.PERCENT);
		
		desiredTarget = joystick;
	}

	public void brake() {
		angleControl(this.getCurrentAngle());
	}

	public void zero() {
		setControlMode(FourBarControlMode.ZEROING);

		desiredTarget = ZEROING_VELOCITY;

		lastTime = RobotController.getFPGATime();
		lastError = desiredTarget - fourBarMotor.getSelectedSensorVelocity();
	}

	public void angleControl(double angle) {
		setControlMode(FourBarControlMode.POSITION);

		desiredTarget = angle;

		lastTime = RobotController.getFPGATime();
		lastError = desiredTarget - this.getCurrentAngle();
	}

	public boolean getLimitSwitch()
	{
		return !limitSwitch.get();
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
			Log.info("CmdZero", "Four Bar zeroed.");
		}

		@Override
		protected void interrupted() {
			Log.info("CmdZero", "Four Bar zeroing interrupted.");
		}
	}

	public class CmdAngleControl extends Command
	{
		FourBarState angleState;
		private double error;

		public CmdAngleControl(FourBarState angleState)
		{
			super(2);
			this.angleState = angleState;
		}

		@Override
		protected void initialize()
		{
			setState(angleState);
			Log.debug("CmdSetFourBarPosition", "Changing state to " + angleState.name());
		}

		@Override
		protected void end() {
			powerControl(0);
			Log.debug("CmdSetFourBarPosition", "Lift at desired height of " + angleState.targetAngle + " degrees.");
		}

		@Override
		protected void interrupted()
		{
			powerControl(0);
			Log.debug("CmdSetFourBarPosition", "Interrupted. Final angle = " + getCurrentAngle() + " degrees.");
		}

		@Override
		protected boolean isFinished()
		{
			error = (getCurrentAngle() - angleState.targetAngle);
			Log.debug("CmdSetFourBarPosition", "Error: " + error + "deg");

			return Math.abs(error) < allowableError || isTimedOut();
		}
	}

}