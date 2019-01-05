package org.team3128.common.drive.onedmotionprofiles;

/**
 * Describes a single continuous move object, in which the Talon SRX accelerates to a specific cruise velocity in order to travel a specific distance.
 * 
 * @author Ronak
 */
public class MotorSegment {
	private double distance, speedFraction;
	
	/**
	 * Creates a new MotorSegment object.
	 * 
	 * @param distance - The distance to be traveled in this move (in native units)
	 * @param speedFraction - The fraction of the cruise velocity for this move to run (from 0.0 to 1.0)
	 */
	public MotorSegment(double distance, double speedFraction) {
		this.distance = distance;
		this.speedFraction = speedFraction;
	}
	
	/**
	 * @return The distance to be traveled in this move (in native units)
	 */
	public double getDistance() {
		return distance;
	}
	
	/**
	 * @return The fraction of the cruise velocity for this move to run (from 0.0 to 1.0)
	 */
	public double getSpeedFraction() {
		return speedFraction;
	}
	
	public String toString() {
		return "Distance: " + distance + " native units. Speed Fraction: " + speedFraction;
	}
}