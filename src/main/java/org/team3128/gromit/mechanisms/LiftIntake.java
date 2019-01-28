package org.team3128.gromit.mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.team3128.common.hardware.misc.Piston;

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
		INTAKE(-1.0, false, "Intake"),
		OUTTAKE(1.0, true, "Outtake");
		private double rollerPower;
		private boolean isClosed;
		private String name;
		
		private LiftIntakeState(double rollerPower, boolean isClosed, String name) {
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
	//private DigitalInput limSwitch;
	private LiftIntakeState state, newState;
	private Piston piston;
	private double invertMultiplier;

	//constructor
	public LiftIntake(VictorSPX intakeMotors, LiftIntakeState state, Piston piston, boolean inverted) {
		this.intakeMotors = intakeMotors;
		//this.limSwitch = limSwitch;
		this.state = state;
		this.piston = piston;		
		
		this.invertMultiplier = (inverted) ? -1 : 1;
		
		this.state = LiftIntakeState.OUTTAKE;
		setState(LiftIntakeState.STOPPED);
	}
	
	public void setState(LiftIntakeState newState) {
		if (state != newState) {
			this.newState = newState;
			
			if(newState.getPistonPosition()) {
				piston.setPistonOn();
			}
			else {
				piston.setPistonOff();
			}
			
			Thread intakeThread = new Thread(() -> {
				if (this.state.equals(LiftIntakeState.INTAKE) && this.newState.equals(LiftIntakeState.STOPPED)) {
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