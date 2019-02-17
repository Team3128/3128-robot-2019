package org.team3128.gromit.mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.util.Log;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
		CARGO_INTAKE(-0.6, true, "Cargo Intake"),
        CARGO_OUTTAKE(1.0, true, "Cargo Outtake"),
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
	DigitalInput cargoBumperSwitch;

	private LiftIntakeState state;
	private Piston demogorgonPiston;

	private Thread cargoThread;

	private static LiftIntake instance = null;
	public static LiftIntake getInstance() {
		if (instance != null) {
			return instance;
		}

		Log.fatal("LiftIntake", "Attempted to get instance before initializtion! Call initialize(...) first.");
		return null;
	}

	public static void initialize(VictorSPX intakeMotors, LiftIntakeState state, Piston demogorgonPiston, DigitalInput cargoBumperSwitch) {
		instance = new LiftIntake(intakeMotors, state, demogorgonPiston, cargoBumperSwitch);
	}

	private LiftIntake(VictorSPX intakeMotors, LiftIntakeState state, Piston demogorgonPiston, DigitalInput cargoBumperSwitch) {
		this.intakeMotors = intakeMotors;
		this.state = state;
		this.demogorgonPiston = demogorgonPiston;
		
		this.cargoBumperSwitch = cargoBumperSwitch;

		cargoThread = new Thread(() -> {			
			while (true) {
				if (this.state.rollerPower > 0 || this.state.rollerPower < 0 && !this.getCargoBumper()) {
					this.setIntakePower(this.state.rollerPower);
				}
				else {
					this.setIntakePower(0);
				}

				SmartDashboard.putBoolean("Cargo Bumper Switch", this.getCargoBumper());

				try {
					Thread.sleep(100);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		});
		cargoThread.start();
	}
	
	public void setState(LiftIntakeState newState) {
		if (state != newState) {			
			if(newState.getPistonPosition()) {
				demogorgonPiston.setPistonOn();
			}
			else {
				demogorgonPiston.setPistonOff();
			}
			
			state = newState;
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
}