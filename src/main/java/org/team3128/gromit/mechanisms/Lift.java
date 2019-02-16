package org.team3128.gromit.mechanisms;

import org.team3128.common.util.Constants;
import org.team3128.common.util.Log;
import org.team3128.common.util.units.Length;

//import org.team3128.gromit.mechanisms.Intake.IntakeState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Control system for the two-stage, continuous elevator-style lift mechanism. The
 * {@link FourBar} is mounted to the first stage of the list.
 * 
 * @author Chris, Jude, Tygan
 * 
 */

public class Lift
{
	public enum LiftHeightState {
		/*
		these need to be defined better
		*/
		BASE(0 * Length.ft),

		INTAKE_FLOOR_CARGO(6.3 * Length.in), //first raise for ball intake
		HOLD_CARGO(12 * Length.in), //second raise for ball intake

		LOW_CARGO(36 * Length.in),
		MID_CARGO(59 * Length.in),
		TOP_CARGO(64 * Length.in),
		
        LOW_HATCH(0 * Length.ft), //same for rocket and cargo and loading station
        MID_HATCH(0 * Length.ft),
		TOP_HATCH(0 * Length.ft),
		
        LOADING_SHIP_CARGO(0 * Length.ft),
        LOADING_SHIP_HATCH(0 * Length.ft);

		public double targetHeight;

		private LiftHeightState(double height)
		{
			this.targetHeight = height;
		}
	}

	public enum LiftControlMode {
		PERCENT(2, "Percent Output"),
		POSITION_UP(0, "Position (Up)"),
		POSITION_DOWN(1, "Position (Down)");

		private int pidSlot;
		private String name;

		private LiftControlMode(int pidSlot, String name)
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

	public void setControlMode(LiftControlMode mode) {
		if (mode != controlMode)
		{
			controlMode = mode;
			Log.debug("Lift", "Setting control mode to " + mode.getName() + ", with PID slot " + mode.getPIDSlot());
			liftMotor.selectProfileSlot(mode.getPIDSlot(), 0);
		}
	}

	/**
	 * The native units that results in lift movement of 1 centimeter.
	 * 
	 * MAX ENCODER POSITION = 51,745 native units
	 * MAX HEIGHT = 78.5 inches
	 */
	public final double ratio = 51745 / (78.8 * Length.in);
	public double error;

	// Physical Components
	TalonSRX liftMotor;
	DigitalInput limitSwitch;

	public LiftControlMode controlMode;
	public LiftHeightState heightState;

	int limitSwitchLocation, liftMaxVelocity;

	// Lift Thread
	Thread liftThread;

	public double brakePower = 0.15;

	public double controlBuffer = 2 * Length.in;

	public double maxHeight = 78 * Length.in;

	public boolean disabled = false;

	public boolean canLower = false;
	public boolean canRaise = true;

	private double desiredTarget = 0;
	private double setPoint = 0;

	public boolean override = false;

	private boolean cmdControlled = false;

	private double lastHeight;


	private static Lift instance = null;
	public static Lift getInstance() {
		if (instance != null) {
			return instance;
		}

		Log.fatal("Lift", "Attempted to get instance before initializtion! Call initialize(...) first.");
		return null;
	}

	public static void initialize(LiftHeightState state, TalonSRX liftMotor, DigitalInput softStopLimitSwitch, int liftMaxVelocity) {
		instance = new Lift(state, liftMotor, softStopLimitSwitch, liftMaxVelocity);
	}

	private Lift(LiftHeightState state, TalonSRX liftMotor, DigitalInput softStopLimitSwitch, int liftMaxVelocity) {
		this.liftMotor = liftMotor;
		this.heightState = state;

		this.limitSwitch = softStopLimitSwitch;
		this.liftMaxVelocity = liftMaxVelocity;

		setControlMode(LiftControlMode.PERCENT);

		liftMotor.configMotionCruiseVelocity((int) (0.9 * liftMaxVelocity), Constants.CAN_TIMEOUT);
		liftMotor.configMotionAcceleration((int) (1.5 * liftMaxVelocity), Constants.CAN_TIMEOUT);

		liftMotor.configOpenloopRamp(0.2, Constants.CAN_TIMEOUT);

		liftThread = new Thread(() ->
		{
			double target = 0;

			while (true)
			{
				if (this.getLimitSwitch())
				{
					this.liftMotor.setSelectedSensorPosition(limitSwitchLocation, 0, Constants.CAN_TIMEOUT);
				}

				if (this.disabled)
				{
					this.liftMotor.set(ControlMode.PercentOutput, 0);
				}
				else {
					target = 0;

					this.canRaise = this.getCurrentHeight() < this.maxHeight - this.controlBuffer;
					this.canLower = this.getCurrentHeight() > 0;

					if (this.controlMode == LiftControlMode.PERCENT) {
						if (this.override) {
							target = this.desiredTarget;
							this.liftMotor.set(ControlMode.PercentOutput, target);
						}
						else {
							if (this.desiredTarget > 0 && this.canRaise) {
								target = this.desiredTarget;
							}
							else if (this.desiredTarget < 0 && this.canLower) {
								target = 0.7 * this.desiredTarget;
							}

							if ((Math.abs(target) < 0.1 && this.getCurrentHeight() >= 3 * Length.in)) {
								target = this.brakePower;
							}

							if (Math.abs(target - this.setPoint) > 0.0001) {
								this.liftMotor.set(ControlMode.PercentOutput, target);

								this.setPoint = target;
							}
						}
					}
					// else if (!this.cmdControlled) {
					// 	if (Math.abs(getCurrentHeight() - heightState.targetHeight) < 4 * Length.in && 
					// 	    Math.abs(getCurrentHeight() - lastHeight) < 0.1 * Length.in) {
					// 		powerControl(0);
					// 	}
					// }
				}

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

		liftThread.start();
	}

	public double getCurrentHeight() {
		return liftMotor.getSelectedSensorPosition(0) / ratio;
	}

	public void setState(LiftHeightState liftState)
	{
		heightState = liftState;

		if (Math.abs(getCurrentHeight() - heightState.targetHeight) > 4 * Length.in) {
			if (getCurrentHeight() < heightState.targetHeight)
			{
				setControlMode(LiftControlMode.POSITION_UP);
			}
			else
			{
				setControlMode(LiftControlMode.POSITION_DOWN);
			}
	
			lastHeight = getCurrentHeight();
			Log.info("Lift", "Setting height to " + heightState.targetHeight + " cm.");
			liftMotor.set(ControlMode.MotionMagic, heightState.targetHeight * ratio);
		}
	}

	public void powerControl(double joystick)
	{
		setControlMode(LiftControlMode.PERCENT);

		desiredTarget = joystick;
	}

	public boolean getLimitSwitch()
	{
		return !limitSwitch.get();
	}

	public class CmdZero extends Command {
		public CmdZero() {
			super(0.2);
		}

		@Override
		protected void initialize() {
			override = true;
			powerControl(-0.2);
		}

		@Override
		protected boolean isFinished()
		{
			return getLimitSwitch() || isTimedOut();
		}

		@Override
		protected void end() {
			override = false;
			liftMotor.setSelectedSensorPosition(limitSwitchLocation, 0, Constants.CAN_TIMEOUT);
		}

		@Override
		protected void interrupted() {
			end();
		}
	}

	public class CmdHeightControl extends Command
	{
		final static double MOVEMENT_ERROR_THRESHOLD = 6 * Length.in;
		final static double ERROR_PLATEAU_THRESHOLD = 0.01 * Length.in;

		final static int ERROR_PLATEAU_COUNT = 10;

		int plateauCount;

		LiftHeightState heightState;

		public CmdHeightControl(LiftHeightState heightState)
		{
			super(3);
			this.heightState = heightState;
		}

		@Override
		protected void initialize()
		{
			cmdControlled = true;

			setState(heightState);
			Log.debug("CmdSetLiftPosition", "Changing state to " + heightState.name());
		}

		@Override
		protected void end() {
			powerControl(0);
			cmdControlled = false;
			Log.debug("CmdSetLiftPosition", "Lift at desired height of " + (heightState.targetHeight / Length.in) + " inches.");
		}

		@Override
		protected void interrupted()
		{
			powerControl(0);
			cmdControlled = false;
			Log.debug("CmdSetLiftPosition", "Interrupted. Final height = " + (getCurrentHeight() / Length.in) + " inches.");
		}

		@Override
		protected boolean isFinished()
		{
			error = getCurrentHeight() - heightState.targetHeight;

			Log.debug("CmdSetLiftPosition", "Error: " + (error / Length.in) + " inches");

			if (Math.abs(error) > 6 * Length.in) return false;

			if (Math.abs(error) < MOVEMENT_ERROR_THRESHOLD) {
				plateauCount += 1;
			}
			else {
				plateauCount = 0;
			}

			if (plateauCount > ERROR_PLATEAU_COUNT) return true;

			return isTimedOut();
		}
	}
}