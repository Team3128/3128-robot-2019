package org.team3128.gromit.mechanisms;

import org.team3128.common.util.Constants;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.units.Angle;
import org.team3128.common.hardware.navigation.Gyro;
import org.team3128.common.hardware.navigation.NavX;

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
    public final double ratio = 4550 / (180 * Angle.DEGREES);
	public double error;

	public Gyro gyro = new NavX();
	private final double allowableError = 2 * Angle.DEGREES;

    public enum FourBarState {
		ZERO(90*Angle.DEGREES), //DEBUG
		SHIP_AND_LOADING(-55 * Angle.DEGREES),
		// HATCH_DROP_SHIP_LOADING(-53 * Angle.DEGREES),

		ROCKET_LOW(-67 * Angle.DEGREES), 
		CARGO_MID(64 * Angle.DEGREES),
		// HATCH_DROP_ROCKET_LOW(-65 * Angle.DEGREES),
		
		CARGO_INTAKE(-22 * Angle.DEGREES),
		//17
		//HATCH_LOW(15 * Angle.DEGREES),
		HATCH_LOW(-60 * Angle.DEGREES),
		//64
		HATCH_HIGH(54 * Angle.DEGREES),
		CARGO_HIGH(76 * Angle.DEGREES);
		//68
		public double targetAngle;

        private FourBarState(double angle){
            this.targetAngle = angle;
		}
	}
	
	public enum FourBarControlMode {
		PERCENT(1, "Percent Output"),
		POSITION(1, "Position");
		// BRAKE(0, "Brake");

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
	public double peakBrakePower = 0.15;
	private double brakeTrigFudge = 0.095;

	public double maxAngle = +90.0 * Angle.DEGREES;
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

	public static void initialize(TalonSRX fourBarMotor, FourBarState state, DigitalInput limitSwitch, double limitSwitchAngle, int maxVelocity) {
		instance = new FourBar(fourBarMotor, state, limitSwitch, limitSwitchAngle, maxVelocity);
	}

	private FourBar(TalonSRX fourBarMotor, FourBarState state, DigitalInput limitSwitch, double limitSwitchAngle, int maxVelocity) {
		this.fourBarMotor = fourBarMotor;
		this.state = state;

		this.limitSwitch = limitSwitch;
		this.limitSwitchAngle = limitSwitchAngle;
		this.maxVelocity = maxVelocity;

		powerControl(0);
				
		fourBarThread = new Thread(() ->
		{
			double target = 0;

			double setPoint = 0;

			double kP;
			double kD;

			while (true)
			{
				if (this.getLimitSwitch())
				{
					this.setCurrentAngle(limitSwitchAngle);
				}

				if (this.disabled)
				{
					this.fourBarMotor.set(ControlMode.PercentOutput, 0);
				}
				else {
					target = 0;

					if (this.controlMode == FourBarControlMode.PERCENT) {
						if (this.override) {
							target = desiredTarget;
							this.fourBarMotor.set(ControlMode.PercentOutput, target);
						}
						else {
							this.canRaise = this.getCurrentAngle() < this.maxAngle - 2 * Angle.DEGREES;
							this.canLower = this.getCurrentAngle() > this.minAngle + 1 * Angle.DEGREES;

							if (desiredTarget > 0 && this.canRaise) {
								target = 0.7 * getAdjustedTarget(desiredTarget);
							}
							else if (desiredTarget < 0 && this.canLower) {
								target = 0.4 * getAdjustedTarget(desiredTarget);
							}

							if ((Math.abs(target) < 0.0001 && this.canRaise && this.canLower)) {
								//this.brakeControl();
								this.brake();
							}
						}
					}
					else if (this.controlMode == FourBarControlMode.POSITION) {
						lastError = this.error;
						this.error = desiredTarget - this.getCurrentAngle();

						// if (Math.abs(this.error) < 0.2) {
						// 	this.brakeControl();
						// }

						if (this.error > 0) {
							kP = 0.285;
						}
						else {
							kP = 0.01;
						}

						kD = 0*0.05;
						target = /*gyro.getPitch() +*/ this.getFeedForwardPower() + kP * this.error + kD * (this.error - lastError) * 1000000 / (RobotController.getFPGATime() - this.lastTime);
						this.lastTime = RobotController.getFPGATime();
					}

					if (Math.abs(target - setPoint) > 0.0001) {
						this.fourBarMotor.set(ControlMode.PercentOutput, target);

						setPoint = target;
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
		fourBarMotor.setSelectedSensorPosition((int) (angle * ratio), 0, Constants.CAN_TIMEOUT);
	}

    public void setState(FourBarState fourBarState)
	{
		state = fourBarState;

		Log.info("FourBar", "Going to " + state.targetAngle + " degrees.");
		angleControl(state.targetAngle);
	}
	
	private double getFeedForwardPower() {
		return peakBrakePower * (brakeTrigFudge + (1 - brakeTrigFudge)*RobotMath.cos(getCurrentAngle()));
	}
    
	public void powerControl(double joystick)
	{
		setControlMode(FourBarControlMode.PERCENT);
		
		desiredTarget = joystick;
	}

	// public void brakeControl(){
	// 	if (controlMode != FourBarControlMode.BRAKE) {
	// 		fourBarMotor.set(ControlMode.Position, (int) (getCurrentAngle() * ratio));
	// 	}
	// 	setControlMode(FourBarControlMode.BRAKE);
	// }

	public void brake() {
		angleControl(this.getCurrentAngle());
	}

	public void angleControl(double angle) {
		setControlMode(FourBarControlMode.POSITION);

		desiredTarget = angle;

		lastTime = 0;
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
			override = true;
			powerControl(0.2);
		}

		@Override
		protected boolean isFinished()
		{
			return getLimitSwitch() || isTimedOut();
		}

		@Override
		protected void end() {
			override = false;
			setCurrentAngle(limitSwitchAngle);
		}

		@Override
		protected void interrupted() {
			end();
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