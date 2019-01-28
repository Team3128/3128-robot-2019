package org.team3128.gromit.mechanisms;

import org.team3128.common.util.Constants;
import org.team3128.common.util.Log;
import org.team3128.common.util.units.Length;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;


public class FourBar
{
    public final double ratio = 19100.0 / (76 * Length.in); //to be calculated, convert angle to native units

    public enum FourBarState{
        BALL_INTAKE(0.0), 
        HATCH_PICKUP(10.0),
    //    LOW_DROP_OFF(40.0), same as hatch pickup 
        HIGH_DROP_OFF(120.0);

        public double targetAngle;

        private FourBarState(double angle){
            this.targetAngle = angle;
        }
    }

    TalonSRX fourBarMotor;

    public void setState(FourBarState fourBarState)
	{
		if (state != fourBarState)
		{
			if (fourBarState.targetAngle < state.targetAngle)
			{
				setControlMode(FourBarControlMode.ANGLE_DOWN);
			}
			else
			{
				setControlMode(FourBarControlMode.ANGLE_UP);
			}
			state = fourBarState;
			Log.info("FourBar", "Going to " + state.targetHeight + " degrees.");
			fourBarMotor.set(ControlMode.MotionMagic, state.targetAngle * ratio);
		}
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

		public CmdSetLiftPosition(fourBarState angleState)
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
			return isTimedOut() || Math.abs(fourBarMotor.getSelectedSensorPosition(0) - (int)(fourBarState.targetAngle * ratio)) < 300;
		}
	}

}