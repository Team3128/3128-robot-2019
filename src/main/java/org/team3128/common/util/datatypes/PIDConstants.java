package org.team3128.common.util.datatypes;

/**
 * Class to store PID constants.
 * @author Jamie
 *
 */
public class PIDConstants
{
	public double kF, kP, kI, kD;
	
	public PIDConstants(double kF, double kP, double kI, double kD)
	{
		this.kF = kF;

		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}
}