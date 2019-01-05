package org.team3128.common.util;

import org.team3128.common.util.datatypes.PIDConstants;


public class VelocityPID
{
	
	double lastVelocityError = 0;
	
	double errorSum = 0;
	
	PIDConstants pidConstants;
	
	double desiredVelocity;
	
	double storedOutput;
	
	public VelocityPID(PIDConstants pidConstants)
	{
		this.pidConstants = pidConstants;
	}
	
	public void setDesiredVelocity(double velocity)
	{
		desiredVelocity = velocity;
	}
	
	/**
	 * Reset the internal sum of the error.
	 * This is a good thing to do if the robot has been sitting for some time and building up error.
	 */
	public void resetIntegral()
	{
		errorSum = 0;
	}
	
	/**
	 * Does another iteration of the PID control calculation
	 * @param currentVelocity
	 */
	public void update(double currentVelocity)
	{
		double error = desiredVelocity - currentVelocity;
		
		errorSum += error;
		
		double errorDerivative = error - lastVelocityError;
		
		storedOutput = (pidConstants.getkP() * error) + (pidConstants.getkI() * errorSum) + (pidConstants.getkD() * errorDerivative) + storedOutput;
		
		lastVelocityError = error;
		
	}
	
	/**
	 * 
	 * @return The number to output to apply the PID correction.
	 * 
	 * This function does not do any calculating, you have to call update() first
	 */
	public double getOutput()
	{
		return storedOutput;
	}
}
