package org.team3128.gromit.mechanisms;

import org.team3128.common.util.Constants;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.units.Length;
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
    public final double ratio = 4000 / 180; //to be calculated, convert angle to native units
	public double error, currentAngle;

    public enum FourBarState {
        CARGO_INTAKE(0.0 * Angle.DEGREES), 
        LOW(10.0 * Angle.DEGREES),
        HIGH(120.0 * Angle.DEGREES);

		public double targetAngle;

        private FourBarState(double angle){
            this.targetAngle = angle;
		}
	}
	
	public enum FourBarControlMode
	{
		PERCENT(2, "Percent Output"),
		POSITION_UP(0, "Position (Up)"),
		POSITION_DOWN(1, "Position (Down)");

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

	public void setControlMode(FourBarControlMode mode)
	{
		if (mode != controlMode)
		{
			controlMode = mode;
			Log.debug("setControlMode", "Setting to " + mode.name());
			fourBarMotor.selectProfileSlot(mode.getPIDSlot(), 0);
			System.out.println(mode.getPIDSlot());
		}
	}

	TalonSRX fourBarMotor;
	FourBarControlMode controlMode;
	FourBarState state;
	
	private Thread fourBarThread;
	public DigitalInput limitSwitch;

	int limitSwitchLocation, maxVelocity;

	public double peakBreakPower = 0.15;
	public double breakAngle = -80 * Angle.DEGREES;

	public int controlBuffer = 200;

	public double maxAngle = 85.0 * Angle.DEGREES;

	public boolean disabled = false;

	public boolean canLower = false;
	public boolean canRaise = true;

	private double desiredTarget = 0;
	private double setPoint = 0;

	public boolean override = false;

	
	private static FourBar instance = null;
	public static FourBar getInstance() {
		if (instance != null) {
			return instance;
		}

		Log.fatal("FourBar", "Attempted to get instance before initializtion! Call initialize(...) first.");
		return null;
	}

	public static void initialize(TalonSRX fourBarMotor, FourBarState state) {
		instance = new FourBar(fourBarMotor, state);
	}

	private FourBar(TalonSRX fourBarMotor, FourBarState state) {
		this.fourBarMotor = fourBarMotor;
		this.state = state;
				
		fourBarThread = new Thread(() ->
		{
			double target = 0;

			while (true)
			{
				if (this.getLiftSwitch())
				{
					this.fourBarMotor.setSelectedSensorPosition(limitSwitchLocation, 0, Constants.CAN_TIMEOUT);
				}

				if (this.disabled)
				{
					this.fourBarMotor.set(ControlMode.PercentOutput, 0);
				}
				else {
					target = 0;

					this.canRaise = this.fourBarMotor.getSelectedSensorPosition(0) < this.maxAngle - this.controlBuffer;
					this.canLower = this.fourBarMotor.getSelectedSensorPosition(0) > this.controlBuffer;

					if (this.controlMode == FourBarControlMode.PERCENT) {
						if (this.override) {
							target = this.desiredTarget;
							this.fourBarMotor.set(ControlMode.PercentOutput, target);
						}
						else {
							if (this.desiredTarget > 0 && this.canRaise) {
								target = this.desiredTarget;
							}
							else if (this.desiredTarget < 0 && this.canLower) {
								target = 0.7 * this.desiredTarget;
							}

							if ((Math.abs(target) < 0.1
									&& this.getCurrentAngle() >= this.breakAngle)) {
								target = this.peakBreakPower * RobotMath.cos(this.getCurrentAngle());
							}

							if (Math.abs(target - this.setPoint) > 0.0001) {
								this.fourBarMotor.set(ControlMode.PercentOutput, target);

								this.setPoint = target;
							}
						}
					}
				}

				this.currentAngle = this.getCurrentAngle();
				double targetHeight = this.state.targetAngle;

				this.error = Math.abs(currentAngle - targetHeight);

				try
				{
					Thread.sleep(100);
				}
				catch (InterruptedException e)
				{
					e.printStackTrace();
				}

			}
		});
		//fourBarThread.start();
	}

	public double getCurrentAngle() {
		return fourBarMotor.getSelectedSensorPosition(0) / ratio;
	}

    public void setState(FourBarState fourBarState)
	{
		if (state != fourBarState)
		{
			if (fourBarState.targetAngle < state.targetAngle)
			{
				setControlMode(FourBarControlMode.POSITION_DOWN);
			}
			else
			{
				setControlMode(FourBarControlMode.POSITION_UP);
			}
			state = fourBarState;
			Log.info("FourBar", "Going to " + state.targetAngle + " degrees.");
			fourBarMotor.set(ControlMode.MotionMagic, state.targetAngle * ratio);
		}
    }
    
	public void powerControl(double joystick)
	{
		setControlMode(FourBarControlMode.PERCENT);
		
		desiredTarget = joystick;
	}

	public boolean getLiftSwitch()
	{
		return false;
		//return limitSwitch.get();
	}
	
	public class CmdForceZeroFourBar extends Command {
		private boolean done = false;

		public CmdForceZeroFourBar() {
			super(0.5);
		}

		@Override
		protected void initialize() {
			fourBarMotor.setSelectedSensorPosition(limitSwitchLocation, 0, Constants.CAN_TIMEOUT);
			state = FourBarState.CARGO_INTAKE;
			done = true;
		}

		@Override
		protected boolean isFinished()
		{
			return done || isTimedOut();
		}
	}

	public class CmdSetFourBarPosition extends Command
	{
		FourBarState angleState;

		public CmdSetFourBarPosition(FourBarState angleState)
		{
			super(2);
			this.angleState = angleState;
		}

		@Override
		protected void initialize()
		{
			setState(angleState);
			Log.debug("CmdSetLiftPosition", "Changing state to " + angleState.name());
			Log.debug("CmdSetLiftPosition", "Target: " + fourBarMotor.getClosedLoopTarget(0));
		}

		@Override
		protected void execute() {
			Log.debug("CmdSetLiftPosition", "Error: " + (fourBarMotor.getSelectedSensorPosition(0) - (int)(angleState.targetAngle * ratio)));
		}

		@Override
		protected void end() {
			powerControl(0);
			Log.debug("CmdSetLiftPosition", "Lift at desired height of " + angleState.targetAngle);
		}

		@Override
		protected void interrupted()
		{
			Log.debug("Lift and Intake", "Ending, was interrupted.");
			end();
		}

		@Override
		protected boolean isFinished()
		{
			return isTimedOut() || Math.abs(fourBarMotor.getSelectedSensorPosition(0) - (int)(angleState.targetAngle * ratio)) < 300;
		}
	}

}