package org.team3128.common.drive.onedmotionprofiles;

/**
 * Custom wrapper to hold motion profile points, of which the important components are the duration, position, and velocity.
 * 
 * @author Ronak
 */
public class ProfilePoint {
	private int position;
	private double velocity, acceleration;
	
	/**
	 * Creates a new ProfilePoint object.
	 * 
	 * @param position - The target location for this profile point (in native units)
	 * @param velocity - The target velocity for this profile point (in native units/second)
	 * @param acceleration
	 */
	public ProfilePoint(int position, double velocity, double acceleration) {		
		this.position = position;
		this.velocity = velocity;
		this.acceleration = acceleration;
	}
	
	/**
	 * @return The target location for this profile point (in native units)
	 */
	public int getPosition() {
		return position;
	}
	
	/**
	 * @return The target velocity for this profile point (in native units/second)
	 */
	public double getVelocity() {
		return velocity;
	}
	
	public String toString() {
		//return "pos: " + position + " nu; velocity: " + velocity;
		return position + " " + velocity + " " + acceleration;
	}
}