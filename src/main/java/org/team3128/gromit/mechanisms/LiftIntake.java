package org.team3128.gromit.mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.util.Log;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Control system for the mechanism controlling mechanisms 
 * 
 * @author Chris, Jude, Tygan
 * 
 */

public class LiftIntake {
    public enum LiftIntakeState
	{
		STOPPED(0, true, "Stopped"),
		BALL_INTAKE(-1.0, true, "Ball Intake"),
        BALL_OUTTAKE(1.0, true, "Ball Outtake"),
        HATCH_INTAKE(0.0, false, "Hatch Panel Intake");

		private double rollerPower;
		private boolean isClosed;
		private String name;
		
		private LiftIntakeState(double rollerPower, boolean isClosed, String name) {
			this.rollerPower = rollerPower;
			this.isClosed = isClosed;
			this.name = name;
		}

		public double getRollerPower() {
			return rollerPower;
		}

		public boolean getPistonPosition() {
			return isClosed;
		}
		
		public String getName() {
			return name;
        }
	}
	
    VictorSPX intakeMotors;
	private LiftIntakeState state;
	private Piston demogorgonPiston;
	private double invertMultiplier;

	private static LiftIntake instance = null;
	public static LiftIntake getInstance() {
		if (instance != null) {
			return instance;
		}

		Log.fatal("LiftIntake", "Attempted to get instance before initializtion! Call initialize(...) first.");
		return null;
	}

	public static void initialize(VictorSPX intakeMotors, LiftIntakeState state, Piston demogorgonPiston, boolean inverted) {
		instance = new LiftIntake(intakeMotors, state, demogorgonPiston, inverted);
	}

	private LiftIntake(VictorSPX intakeMotors, LiftIntakeState state, Piston demogorgonPiston, boolean inverted) {
		this.intakeMotors = intakeMotors;
		this.state = state;
		this.demogorgonPiston = demogorgonPiston;		
		
		this.invertMultiplier = (inverted) ? -1 : 1;
		
		this.state = LiftIntakeState.BALL_OUTTAKE;
		setState(LiftIntakeState.STOPPED);
	}
	
	public void setState(LiftIntakeState newState) {
		if (state != newState) {			
			if(newState.getPistonPosition()) {
				demogorgonPiston.setPistonOn();
			}
			else {
				demogorgonPiston.setPistonOff();
			}
			
			setIntakePower(newState.getRollerPower());

			state = newState;
		}
	}
	
	private void setIntakePower(double power) {
		intakeMotors.set(ControlMode.PercentOutput, invertMultiplier * power);
	}

	public class CmdSetLiftIntakeState extends Command {
		LiftIntakeState desiredState;

		public CmdSetLiftIntakeState(LiftIntakeState state) {
			super(0.1);

			desiredState = state;
		}

		@Override
		protected void initialize()
		{
			setState(desiredState);
		}

		@Override
		protected boolean isFinished()
		{
			return isTimedOut();
		}
	}
}