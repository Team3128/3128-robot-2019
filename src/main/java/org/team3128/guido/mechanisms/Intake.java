package org.team3128.guido.mechanisms;

import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.util.Log;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * Control system for the intake mechanism V.1
 * 
 * @author Eli, Adham
 * 
 */

public class Intake {
	public enum IntakeState {
		STOPPED(-0.2, true, "Stopped"),
		INTAKE(-1.0, false, "Intake"),
		OUTTAKE(1.0, true, "Outtake"),
		SOFT_DROP(0.5, false, "Soft Drop");

		private double rollerPower;
		private boolean isClosed;
		private String name;
		
		private IntakeState(double rollerPower, boolean isClosed, String name) {
			this.rollerPower = rollerPower;
			this.isClosed = isClosed;
			this.name= name;
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
	private IntakeState state, newState;
	private Piston piston;
	private double invertMultiplier;

	private static Intake instance = null;
	public static Intake getInstance() {
		if (instance != null) {
			return instance;
		}

		Log.fatal("Intake", "Attempted to get instance before initializtion! Call initialize(...) first.");
		return null;
	}

	public static void initialize(VictorSPX intakeMotors, IntakeState state, Piston piston, boolean inverted) {
		instance = new Intake(intakeMotors, state, piston, inverted);
	}

	private Intake(VictorSPX intakeMotors, IntakeState state, Piston piston, boolean inverted) {
		this.intakeMotors = intakeMotors;
		this.state = state;
		this.piston = piston;		
		
		this.invertMultiplier = (inverted) ? -1 : 1;
		
		this.state = IntakeState.OUTTAKE;
		setState(IntakeState.STOPPED);
	}
	
	public void setState(IntakeState newState) {
		if (state != newState) {
			this.newState = newState;
			
			if(newState.getPistonPosition()) {
				piston.setPistonOn();
			}
			else {
				piston.setPistonOff();
			}
			
			Thread intakeThread = new Thread(() -> {
				if (this.state.equals(IntakeState.INTAKE) && this.newState.equals(IntakeState.STOPPED)) {
					try
					{
						Thread.sleep(1000);
					}
					catch (InterruptedException e)
					{
						e.printStackTrace();
					}
				}
				
				setIntakePower(this.newState.getRollerPower());
				this.state = this.newState;
			});
			intakeThread.start();
			
			
		}
	}
	
	public synchronized void setIntakePower(double power) {
		intakeMotors.set(ControlMode.PercentOutput, invertMultiplier * power);
	}
}