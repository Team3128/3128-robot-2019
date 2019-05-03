package org.team3128.gromit.mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.team3128.common.generics.Mechanism;
import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.util.Log;
import org.team3128.gromit.mechanisms.Lift.LiftHeightTarget;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Control system for the mechanism controlling mechanisms 
 * 
 * @author Chris, Jude, Tygan
 * 
 */

public class LiftIntake extends Mechanism {
	@Override
	public String getTag() {
		return "LiftIntake";
	}

    public enum LiftIntakeState
	{
		CARGO_INTAKE(true, "Cargo Intake"),
		CARGO_OUTTAKE(true, "Cargo Outtake"),

		CARGO_HOLDING(true, "Cargo Holding"),
		
		DEMOGORGON_RELEASED(false, "Demogorgon Released"),
        DEMOGORGON_HOLDING(true, "Demogorgon Holding");

		private boolean demogorgonPistonState;
		private String name;
		
		private LiftIntakeState(boolean demogorgonPistonState, String name) {
			this.demogorgonPistonState = demogorgonPistonState;
			this.name = name;
		}

		public boolean getDemogorgonPistonState() {
			return demogorgonPistonState;
		}
		
		public String getName() {
			return name;
        }
	}
	
	VictorSPX intakeMotors;
	DigitalInput cargoBumperSwitch;

	private LiftIntakeState newState;
	public LiftIntakeState currentState;

	private Piston demogorgonPiston;

	public boolean bumped = false;


	private static LiftIntake instance = null;
	public static LiftIntake getInstance() {
		if (instance != null) {
			return instance;
		}

		Log.fatal(instance, "Attempted to get instance before initializtion! Call initialize(...) first.");
		return null;
	}

	public static void initialize(VictorSPX intakeMotors, LiftIntakeState state, Piston demogorgonPiston, DigitalInput cargoBumperSwitch) {
		instance = new LiftIntake(intakeMotors, state, demogorgonPiston, cargoBumperSwitch);
	}

	private LiftIntake(VictorSPX intakeMotors, LiftIntakeState state, Piston demogorgonPiston, DigitalInput cargoBumperSwitch) {
		this.intakeMotors = intakeMotors;
		this.demogorgonPiston = demogorgonPiston;
		
		this.cargoBumperSwitch = cargoBumperSwitch;

		setState(state);	
	}

	@Override
	protected void controlLoop() {
		if (newState == null) {
			if (currentState == LiftIntakeState.CARGO_INTAKE) {
				if (getCargoBumper()) {
					bumped = true;
					setIntakePower(-0.2);
				}
				else {
					setIntakePower(-1.0);
				}
			}
			else if (currentState == LiftIntakeState.CARGO_HOLDING && bumped) {
				if (getCargoBumper()) {
					setIntakePower(-0.2);
				}
				else {
					setIntakePower(-1.0);
				}
			}
		}
		else {
			if (this.newState == LiftIntakeState.CARGO_INTAKE) {
				setIntakePower(-1.0);

				bumped = false;
			}
			else if (newState == LiftIntakeState.CARGO_OUTTAKE) {
				setIntakePower(1.0);

				bumped = false;
			}
			else {
				setIntakePower(0.0);
			}

			currentState = newState;
			newState = null;
		}
	}

	@Override
	public void enable() {
		setState(LiftIntakeState.DEMOGORGON_HOLDING);
	}

	@Override
	public void disable() {
		setState(LiftIntakeState.DEMOGORGON_HOLDING);
	}

	@Override
	public void zero() {}
	
	public void setState(LiftIntakeState newState) {
		if (this.currentState != newState) {			
			if (newState.getDemogorgonPistonState()) {
				demogorgonPiston.setPistonOn();
			}
			else {
				demogorgonPiston.setPistonOff();
			}
			
			this.newState = newState;
		}
	}
	
	private void setIntakePower(double power) {
		intakeMotors.set(ControlMode.PercentOutput, power);
	}

	public boolean getCargoBumper() {
		return !this.cargoBumperSwitch.get();
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

	public class CmdRetractHatch extends Command {
		LiftHeightTarget liftState;
		boolean isPickUp = false;
		boolean isDone = false;
		
		public CmdRetractHatch(LiftHeightTarget liftState) {
			this.liftState = liftState;
		}
		
		@Override
		protected void initialize() {
			isPickUp = liftState == LiftHeightTarget.HATCH_INTAKE;
			LiftIntake.getInstance().setState(LiftIntakeState.DEMOGORGON_HOLDING);
			if(isPickUp) {
				Lift.getInstance().heightControl(LiftHeightTarget.HATCH_PULL_UP);
			}
		}

		@Override
		protected boolean isFinished() {
			return true;
		}
		
	}
}