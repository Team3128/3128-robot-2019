package org.team3128.common.drive.onedmotionprofiles;

/**
 * The transition between successive Segment objects, in which the motion profile accelerates trapezoidal between the two (potentially different) cruise velocities.
 * 
 * @author Ronak
 */
public class AccelerationPoint {
	private double delVelocity;
	
	private double maxAcceleration;
	private double rampTime;
	
	private double triangularThreshold;
	
	/**
	 * Creates a new AccelerationPoint
	 * 
	 * @param delVelocity - The net change in velocity at this acceleration interface (in native units / second)
	 */
	public AccelerationPoint(double delVelocity, MotionProfileGenerator parent) {
		this.delVelocity = delVelocity;
		
		maxAcceleration = parent.getMaxAcceleration();
		rampTime = parent.getRampTime();
		
		triangularThreshold = parent.getTriangularThreshold();
	}
	
	public double getRampDuration() {
		if (isTriangular()) {
			return (Math.sqrt(Math.abs(delVelocity) * rampTime / maxAcceleration));
		}
		
		return rampTime;
	}
	
	public double getMaxAccelDuration() {
		if (isTriangular()) return 0;
		
		return (Math.abs(delVelocity) - triangularThreshold) / maxAcceleration;
	}
	
	public double getAcceleratedDistance() {
		if (isTriangular()) {
			return (1 / 3) * (maxAcceleration / rampTime) * Math.pow(getRampDuration(), 3);
		}
		
		return (4 / 3) * maxAcceleration * Math.pow(rampTime, 2) ;
	}
	
	public boolean isTriangular() {
		return (Math.abs(delVelocity) <= triangularThreshold);
	}
	
	public int getSignMultiplier() {
		if (delVelocity > 0) return 1;
		else return -1;
	}
	
	public String toString() {
		return "Delta Velocity: " + delVelocity + " nu/s.";
	}
}
