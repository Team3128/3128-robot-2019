package org.team3128.gromit.mechanisms;

import org.team3128.common.util.Constants;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.units.Angle;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Control system for the four bar
 * 
 * @author Chris, Jude, Tygan
 * 
 */

public class FourBar
{
	/**
	 * The angle in native units equal to 1 degree.
	 */
    public final double ratio = 4550 / (180 * Angle.DEGREES);
	public double error;

	private final double allowableError = 2 * Angle.DEGREES;

    public enum FourBarState {
        CARGO_INTAKE(-70 * Angle.DEGREES), 
        LOW(-40 * Angle.DEGREES),
        HIGH(30 * Angle.DEGREES);

		public double targetAngle;

        private FourBarState(double angle){
            this.targetAngle = angle;
		}
	}
	
	public enum FourBarControlMode {
		PERCENT(2, "Percent Output"),
		POSITION(0, "Position");

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
			Log.debug("FourBar", "Setting control mode to " + mode.name());
			fourBarMotor.selectProfileSlot(mode.getPIDSlot(), 0);
			System.out.println(mode.getPIDSlot());
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
	public double peakBreakPower = 0.2;
	private double breakFudgeTrig = 0.095;

	public double maxAngle = +85.0 * Angle.DEGREES;
	public double minAngle = -85.0 * Angle.DEGREES;

	public boolean disabled = false;

	public boolean canLower = false;
	public boolean canRaise = true;

	private double desiredTarget = 0;
	private double setPoint = 0;

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
				
		fourBarThread = new Thread(() ->
		{
			double target = 0;

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
							target = this.desiredTarget;
							this.fourBarMotor.set(ControlMode.PercentOutput, target);
						}
						else {
							this.canRaise = this.getCurrentAngle() < this.maxAngle - 5 * Angle.DEGREES;
							this.canLower = this.getCurrentAngle() > this.minAngle + 3 * Angle.DEGREES;

							if (this.desiredTarget > 0 && this.canRaise) {
								target = 0.7 * getAdjustedTarget(this.desiredTarget);
							}
							else if (this.desiredTarget < 0 && this.canLower) {
								target = 0.4 * getAdjustedTarget(this.desiredTarget);
							}

							if ((Math.abs(target) < 0.0001 && this.canRaise && this.canLower)) {
								Log.info("FourBar", "Braking...");
								
								this.angleControl(this.getCurrentAngle());
							}
						}
					}
					else if (this.controlMode == FourBarControlMode.POSITION) {
						this.error = desiredTarget - this.getCurrentAngle();

						target = this.getFeedForwardPower() + 0.1 * this.error;
					}

					if (Math.abs(target - this.setPoint) > 0.0001) {
						this.fourBarMotor.set(ControlMode.PercentOutput, target);

						this.setPoint = target;
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
		return peakBreakPower * (breakFudgeTrig + (1 - breakFudgeTrig)*RobotMath.cos(getCurrentAngle()));
	}
    
	public void powerControl(double joystick)
	{
		setControlMode(FourBarControlMode.PERCENT);
		
		desiredTarget = joystick;
	}

	public void angleControl(double angle) {
		setControlMode(FourBarControlMode.POSITION);

		desiredTarget = angle;
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