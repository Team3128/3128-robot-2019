package org.team3128.common.utility.datatypes;

/**
 * Class to store PID constants.
 * 
 * @author Jamie
 *
 */
public class PIDConstants {
	public double kF, kP, kI, kD;

	public PIDConstants(double kF, double kP, double kI, double kD) {
		this.kF = kF;

		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}

	@Override
	public String toString() {
		return "F: " + kF + ", P: " + kP + ", I: " + kI + ", D:" + kD;
	}
}