package org.team3128.gromit.mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.util.Log;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
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
		CARGO_INTAKE(-0.6, true, "Cargo Intake"),
		CARGO_OUTTAKE(1.0, true, "Cargo Outtake"),
		
		DEMOGORGON_RELEASED(0.0, false, "Demogorgon Released"),
        DEMOGORGON_HOLDING(0.0, true, "Demogorgon Holding");

		private double rollerPower;
		private boolean demogorgonPistonState;
		private String name;
		
		private LiftIntakeState(double rollerPower, boolean demogorgonPistonState, String name) {
			this.rollerPower = rollerPower;
			this.demogorgonPistonState = demogorgonPistonState;
			this.name = name;
		}

		public double getRollerPower() {
			return rollerPower;
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
		this.demogorgonPiston = demogorgonPiston;
		
		this.cargoBumperSwitch = cargoBumperSwitch;

		setState(state);

		cargoThread = new Thread(() -> {
			boolean wasBumped = false;
			double bumpTime = 0;

			final double holdTime = 1.0;

			while (true) {
				if (this.state == LiftIntakeState.CARGO_INTAKE) {
					if (this.getCargoBumper()) {
						if (!wasBumped) {
							this.setIntakePower(0.2 * this.state.rollerPower);

							bumpTime = RobotController.getFPGATime() / 1000000.0;
							wasBumped = true;
						}
						else if (RobotController.getFPGATime() / 1000000.0 > bumpTime + holdTime) {
							this.setIntakePower(0);
						}
					}
					else {
						wasBumped = false;
						this.setIntakePower(this.state.rollerPower);
					}
				}
				else {
					this.setIntakePower(0);
				}

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
			if(newState.getDemogorgonPistonState()) {
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