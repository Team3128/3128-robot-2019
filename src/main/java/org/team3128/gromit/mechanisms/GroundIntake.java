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

public class GroundIntake {
    public enum GroundIntakeState
	{
		DEPLOYED(0, true, "Deployed"),
		DEPLOYED_INTAKE(-1.0, false, "Deployed and Intake"),
		RETRACTED(0.0, true, "Retract");
		private double rollerPower;
		private boolean isClosed;
		private String name;
		
		private GroundIntakeState(double rollerPower, boolean isClosed, String name) {
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
	private GroundIntakeState state, newState;
    private Piston leftPiston;
    private Piston rightPiston;
	private double invertMultiplier;

	//constructor
	public GroundIntake(VictorSPX intakeMotors, GroundIntakeState state, Piston leftPiston, Piston rightPiston, boolean inverted) {
		this.intakeMotors = intakeMotors;
		//this.limSwitch = limSwitch;
		this.state = state;
        this.leftPiston = leftPiston;
        this.rightPiston = rightPiston;		
		
		this.invertMultiplier = (inverted) ? -1 : 1;
		
		this.state = GroundIntakeState.RETRACTED;
		setState(GroundIntakeState.RETRACTED);
	}
	
	public void setState(GroundIntakeState newState) {
		if (state != newState) {
			this.newState = newState;
			
			if(newState.getPistonPosition()) {
                leftPiston.setPistonOn();
                rightPiston.setPistonOn();
			}
			else {
                leftPiston.setPistonOff();
				rightPiston.setPistonOff();
			}
			
			Thread intakeThread = new Thread(() -> {
				if (this.state.equals(GroundIntakeState.DEPLOYED_INTAKE) && this.newState.equals(GroundIntakeState.RETRACTED)) {
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