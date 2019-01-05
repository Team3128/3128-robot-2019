package org.team3128.common.hardware.motor.logic;

import org.team3128.common.hardware.motor.MotorLogic;
import org.team3128.common.util.VelocityPID;
import org.team3128.common.util.datatypes.PIDConstants;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

/**
 * Motor control logic that tries to hold the motor at a certain current.
 * It uses P only, and needs an estimate of how much current the motor uses at full power.
 * @author Jamie
 *
 */
public class ConstantCurrentLogic extends MotorLogic
{
	private PowerDistributionPanel panel;
	private int motorPort;
	
	//the calculation for this is pretty much the same as velocity PID.
	//so, we can use the same math object
	private VelocityPID pidCalc;
    
    /**
     * Construct ConstantCurrentLogic
     * @param panel PDP object to use
     * @param motorPort the port number that the motor is in on the PDP.
     */
    public ConstantCurrentLogic(PowerDistributionPanel panel, int motorPort, PIDConstants pidConstants, double feedForward)
    {
        this.panel = panel;
        this.motorPort = motorPort;
        pidCalc = new VelocityPID(pidConstants);
    }
   
    @Override
    /**
     * Set the current to target in amps.
     */
    public synchronized void setControlTarget(double d)
    {
    	pidCalc.setDesiredVelocity(d);
    	

    }

    @Override
    public double speedControlStep(double dt)
    {
        double currentCurrent = panel.getCurrent(motorPort);
        pidCalc.update(currentCurrent);
        return pidCalc.getOutput();
    }
   
    @Override
    public synchronized void reset()
    {
    	pidCalc.setDesiredVelocity(0);
    	pidCalc.resetIntegral();
    }

    @Override
    public boolean isComplete()
    {
    	return false;
    }
}
