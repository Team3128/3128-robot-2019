package org.team3128.common.drive.onedmotionprofiles;

import java.util.ArrayList;
import java.util.List;

/**
 * Generates MotionProfile autonomous move sequences to use with the TalonSRX and the CTRE Magnetic Encoder.
 *  - All distances are in native units (nu)
 *  - All velocities are in native units per second (nu/s)
 *    + NOTE: When actually entering these into Talon SRX motion profiles, divide by ten to get nu/100ms
 *  - All accelerations are in native units per second squared (nu/s^2)
 *  
 * I'm going to be completely honest, this is an experiment. So if you do end up using this, please make sure it works. Thanks.
 *  
 * @author Ronak
 */
public class MotionProfileGenerator {
	public static void main(String[] args) {
		MotionProfileGenerator g = new MotionProfileGenerator(10, 10, 0.600, 30);
		
		g.generateProfile(new MotorSegment(100, 1.0), new MotorSegment(40, 0.5), new MotorSegment(20, 0), new MotorSegment(40, 1.0), new MotorSegment(100, 0.5));
	}
	
	/**
	 * The itp value of the motion profile, the duration of each profile point, in seconds.
	 */
	private double pointDuration;
	
	private double cruiseVelocity, maxAcceleration;
	private double rampTime;
	
	private double triangularThreshold;
	
	/**
	 * Creates a new MotionProfileGenerator
	 * 
	 * @param cruiseVelocity - The velocity at which the profile makes linear moves at (in native units/sec)
	 * @param maxAcceleration - The maximum acceleration of the profile (in native units/sec^2)
	 * @param rampTime - The amount of time for the drive to accelerate from zero to max acceleration (in seconds)
	 */
	public MotionProfileGenerator(double cruiseVelocity, double maxAcceleration, double rampTime, int pointDurationMs) {
		this.cruiseVelocity = cruiseVelocity;
		this.maxAcceleration = maxAcceleration;
		
		this.rampTime = rampTime;
		
		this.pointDuration = pointDurationMs / 1000.0;
		
		triangularThreshold = maxAcceleration * rampTime;
	}
	
	double gRelTime = 0;
	
	double gPosition = 0;
	double gVelocity = 0;
	double gAcceleration = 0;
	
	private void velocityTick(List<ProfilePoint> points) {
		gPosition += gVelocity * pointDuration;
		
		points.add(new ProfilePoint((int) gPosition, gVelocity, gAcceleration));
		
		gRelTime += pointDuration;
	}
	
	private void accelerationTick(List<ProfilePoint> points, AccelerationPoint accelPoint) {
		gVelocity += accelPoint.getSignMultiplier() * gAcceleration * pointDuration;
		gPosition += gVelocity * pointDuration;
		
		points.add(new ProfilePoint((int) gPosition, gVelocity, gAcceleration));
		
		gRelTime += pointDuration;
	}
	
	/**
	 * Generates a sequence of ProfilePoints based on a sequence of Segments. Uses a trapezoidal acceleration curve at each interface to create a smooth motion.
	 * 
	 * @param segments - The array of Segment objects that define successive moves.
	 */
	public List<ArrayList<ProfilePoint>> generateProfile(List<MotorSegment> segments) {
		MotorSegment[] array = (MotorSegment[]) segments.toArray();
		
		return generateProfile(array);
	}
	
	/**
	 * Generates a sequence of ProfilePoints based on a sequence of Segments. Uses a trapezoidal acceleration curve at each interface to create a smooth motion.
	 * 
	 * @param segments - The array of Segment objects that define successive moves.
	 */
	public List<ArrayList<ProfilePoint>> generateProfile(MotorSegment... segments) {
		List<ArrayList<ProfilePoint>> pointsList = new ArrayList<ArrayList<ProfilePoint>>();
		List<AccelerationPoint> accelPoints = new ArrayList<AccelerationPoint>();
		
		for (int i = 0; i < segments.length - 1; i++) {
			MotorSegment first = segments[i];
			MotorSegment second = segments[i+1];
			
			accelPoints.add(new AccelerationPoint(cruiseVelocity * (second.getSpeedFraction() - first.getSpeedFraction()), this));
		}
				
		accelPoints.add(0, new AccelerationPoint(cruiseVelocity * segments[0].getSpeedFraction(), this));
		accelPoints.add(new AccelerationPoint(-1 * cruiseVelocity * segments[segments.length - 1].getSpeedFraction(), this));
		
		
		gRelTime = 0;
		
		gPosition = 0;
		gVelocity = 0;
		gAcceleration = 0;
		
		for (int i = 0; i < segments.length; i++) {
			ArrayList<ProfilePoint> points = new ArrayList<ProfilePoint>();
			
			AccelerationPoint front = accelPoints.get(i);
			MotorSegment seg = segments[i];
			AccelerationPoint back = accelPoints.get(i+1);
			
			double cruiseDistance = seg.getDistance() - front.getAcceleratedDistance() / 2 - back.getAcceleratedDistance() / 2;
			double cruiseDuration = cruiseDistance / cruiseVelocity;
			
			// Begin the acceleration trapezoid if this is the first segment.
			if (i == 0) {
				gRelTime = 0;
				
				while (gRelTime < front.getRampDuration()) {
					gAcceleration += maxAcceleration / rampTime * pointDuration;
					
					accelerationTick(points, front);
				}
				
				
				gRelTime = 0;
				
				if (!back.isTriangular()) {
					gAcceleration = maxAcceleration;
				}
				while (gRelTime < front.getMaxAccelDuration() / 2) {
					accelerationTick(points, front);
				}
			}
			
			// Close off the accleration trapezoid from the previous segment.
			gRelTime = 0;
			
			if (!front.isTriangular()) {
				gAcceleration = maxAcceleration;
			}
			while (gRelTime < front.getMaxAccelDuration() / 2) {
				accelerationTick(points, front);
			}
			
			gRelTime = 0;
			while (gRelTime < front.getRampDuration()) {
				gAcceleration -= maxAcceleration / rampTime * pointDuration;
				
				accelerationTick(points, front);
			}
			
			// Cruise
			gRelTime = 0;
			
			gAcceleration = 0;
			gVelocity = cruiseVelocity * seg.getSpeedFraction();
			while (gRelTime < cruiseDuration) {
				velocityTick(points);
			}
			
			// Accelerate to next Segment
			gRelTime = 0;
			
			while (gRelTime < back.getRampDuration()) {
				gAcceleration += maxAcceleration / rampTime * pointDuration;
				
				accelerationTick(points, back);
			}
			
			
			gRelTime = 0;
			
			if (!back.isTriangular()) {
				gAcceleration = maxAcceleration;
			}
			while (gRelTime < back.getMaxAccelDuration() / 2) {
				accelerationTick(points, back);
			}
			
			// Last Segment Acceleration
			
			if (i == segments.length - 1) {
				gRelTime = 0;
				
				if (!back.isTriangular()) {
					gAcceleration = maxAcceleration;
				}
				while (gRelTime < back.getMaxAccelDuration() / 2) {
					accelerationTick(points, back);
				}
				
				gRelTime = 0;
				while (gRelTime < back.getRampDuration()) {
					gAcceleration -= maxAcceleration / rampTime * pointDuration;
					
					accelerationTick(points, back);
				}
			}
			
			pointsList.add(points);
		}
		
		for (ArrayList<ProfilePoint> points : pointsList) {
			for (ProfilePoint point : points) {
				System.out.println(point);
			}
		}
		
		return pointsList;
	}
	
	/**
	 * @return the velocity at which the profile makes linear moves at (in native units/sec)
	 */
	public double getCruiseVelocity() {
		return cruiseVelocity;
	}
	
	/**
	 * @return the maximum acceleration of the profile (in native units/sec^2)
	 */
	public double getMaxAcceleration() {
		return maxAcceleration;
	}
	
	/**
	 * @return the amount of time for the drive to accelerate from zero to max acceleration (in seconds)
	 */
	public double getRampTime() {
		return rampTime;
	}
	
	/**
	 * @return the maximum velocity for which the acceleration ends up being t, not 
	 */
	public double getTriangularThreshold() {
		return triangularThreshold;
	}
}
