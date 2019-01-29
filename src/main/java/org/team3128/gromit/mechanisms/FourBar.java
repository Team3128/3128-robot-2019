package org.team3128.gromit.mechanisms;

import org.team3128.common.util.Constants;
import org.team3128.common.util.Log;
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
    public final double ratio = 19100.0 / (76 * Length.in); //to be calculated, convert angle to native units

    public enum FourBarState{
        BALL_INTAKE(0.0 * Angle.DEGREES), 
        HATCH_PICKUP(10.0 * Angle.DEGREES),
    //    LOW_DROP_OFF(40.0), same as hatch pickup 
        HIGH_DROP_OFF(120.0 * Angle.DEGREES);

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
	public FourBarControlMode controlMode;
	FourBarState state;
	int limitSwitchLocation;
	private double desiredAngle;
	
	//constructor
	public FourBar(TalonSRX fourBarMotor, FourBarState state) {
		this.fourBarMotor = fourBarMotor;
		this.state = state;
				
		this.state = FourBarState.HATCH_PICKUP;
		setState(FourBarState.HATCH_PICKUP);
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
		
		desiredAngle = joystick;
	}
	
	public class CmdZeroFourBar extends Command {
		private boolean done = false;

		public CmdZeroFourBar() {
			super(0.5);
		}

		@Override
		protected void initialize() {
			fourBarMotor.setSelectedSensorPosition(limitSwitchLocation, 0, Constants.CAN_TIMEOUT);
			state = FourBarState.BALL_INTAKE;
			done = true;
		}

		@Override
		protected boolean isFinished()
		{
			return true || isTimedOut();
		}
	}

	public class CmdSetLiftPosition extends Command
	{
		FourBarState angleState;

		public CmdSetLiftPosition(FourBarState angleState)
		{
			super(3);
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