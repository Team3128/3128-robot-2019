/**
 * @author Adham Elarabawy 
 */
package org.team3128.common.control;

import org.team3128.common.utility.NarwhalUtility;

import edu.wpi.first.wpilibj.Timer;

/**
 * Limits acceleration and jerk
 */
public class RateLimiter {

	private double accelLimit, jerkLimit, latestValue;
	private double currentAccel;
	private double lastTime;

	public RateLimiter(double accel) {
		this(accel, Double.POSITIVE_INFINITY);
	}

	/**
	 *
	 * @param accel Max acceleration in units with an arbitrary time unit. The units
	 *              match whatever you send in update()
	 * @param jerk  Max jerk in units with an arbitrary time unit. The units match
	 *              whatever you send in update()
	 */
	public RateLimiter(double accel, double jerk) {
		this.accelLimit = accel;
		this.jerkLimit = jerk;
		latestValue = 0;
	}

	/**
	 *
	 * @param setpoint Target velocity to accelerate towards
	 * @return Calculated latest value
	 */
	public double update(double setpoint) {
		// Calculate delta T
		double dt = Timer.getFPGATimestamp() - lastTime;
		lastTime = Timer.getFPGATimestamp();

		if (dt > 0.1)
			System.out.println("limiter update took too long (" + setpoint + "," + dt + "," + latestValue + ")");

		double dInput = setpoint - latestValue; // Expected deltaV
		if (dInput == 0)
			return latestValue;

		// Area under triangle at end of trapezoidal motion profile
		// Represents velocity change in the time it takes to reach zero acceleration
		// under current accel and jerk values
		// dV = a*t // Change in velocity = accel * time
		// j = a/t // Jerk is rate in change in acceleration over time
		// t = a/j // Time to reach acceleration a with jerk j
		// dV = a^2 / j
		double area = (Math.pow(currentAccel, 2) / jerkLimit);

		// If expected change in velocity is greater than achieveable with current
		// acceleration, increase acceleration
		// If the expected change in velocity cannot be achieved in time, decrease
		// acceleration
		double dAccel = Math.copySign(jerkLimit * dt, dInput);
		if (Math.abs(dInput) >= area || Math.signum(dInput) != Math.signum(currentAccel))
			currentAccel += dAccel;
		else
			currentAccel -= dAccel;

		currentAccel = NarwhalUtility.coerce(currentAccel, accelLimit, -accelLimit); // Limit currentAccel to accelLimit
		latestValue += currentAccel * dt; // Integrate acceleration into velocity

		// Cap velocity at the target value
		if (Math.signum(setpoint - latestValue) != Math.signum(dInput)) { // Cap velocity at the target value
			latestValue = setpoint;
			currentAccel = 0;
		}

		return latestValue;
	}

	/**
	 *
	 * @param setpoint      What value to accelerate towards
	 * @param dt            How much time has past between iterations
	 * @param remainingDist Distance remaining before complete stop
	 * @return Calculated latest value
	 */
	public double update(double setpoint, double remainingDist) {
		double timeToDecel = getLatestValue() / getAccelLimit();
		double distanceToStop = timeToDecel * getLatestValue();
		if (Math.abs(distanceToStop) >= remainingDist)
			return update(0.0);
		else
			return update(setpoint);
	}

	/**
	 *
	 * @return Current acceleration value
	 */
	public double getAccel() {
		return currentAccel;
	}

	/**
	 *
	 * @return Current maximum jerk value
	 */
	public double getJerkLimit() {
		return jerkLimit;
	}

	/**
	 *
	 * @return Current maximum acceleration value
	 */
	public double getAccelLimit() {
		return accelLimit;
	}

	/**
	 *
	 * @return Latest value calculated
	 */
	public double getLatestValue() {
		return latestValue;
	}

	/**
	 *
	 * @param val Value to set the latest value to
	 */
	public void setLatestValue(double val) {
		latestValue = val;
	}

	/**
	 * Resets the timer
	 */
	public void resetTime() {
		lastTime = Timer.getFPGATimestamp();
	}

	/**
	 * Sets the latest value to 0, current acceleration value to 0, and resets timer
	 */
	public void reset() {
		latestValue = 0;
		currentAccel = 0;
		resetTime();
	}

	/**
	 *
	 * @param accelLimit Desired max acceleration
	 */
	public void setaccelLimit(double accelLimit) {
		this.accelLimit = accelLimit;
		currentAccel = NarwhalUtility.coerce(currentAccel, accelLimit, -accelLimit);
	}

	/**
	 *
	 * @param jerkLimit Desired max Jerk
	 */
	public void setjerkLimit(double jerkLimit) {
		this.jerkLimit = jerkLimit;
	}

	/**
	 *
	 * @param currentAccel Desired acceleration value
	 */
	public void setcurrentAccel(double currentAccel) {
		this.currentAccel = currentAccel;
	}
}
