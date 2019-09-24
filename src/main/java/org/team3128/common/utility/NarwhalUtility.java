/**
 * @author Adham Elarabawy
 */
package org.team3128.common.utility;

public class NarwhalUtility {

	/**
	 * Keeps a value in a range by truncating it.
	 *
	 * @param toCoerce the value to coerce
	 * @param high     the high value of the range
	 * @param low      the low value of the range
	 * @return the coerced value
	 */
	public static double coerce(double toCoerce, double high, double low) {
		if (toCoerce > high) {
			return high;
		} else if (toCoerce < low) {
			return low;
		}
		return toCoerce;
	}

	public static double coercedNormalize(double rawValue, double minInput, double maxInput, double minOutput,
			double maxOutput) {
		if (rawValue < minInput) {
			return minOutput;
		} else if (rawValue > maxInput) {
			return maxOutput;
		}
		double norm = (Math.abs(rawValue) - minInput) / (maxInput - minInput);
		norm = Math.copySign(norm * (maxOutput - minOutput), rawValue) + minOutput;
		return norm;
	}

	/**
	 * Encapsulates Thread.sleep to make code more readable.
	 *
	 * @param millis the time to sleep
	 */
	public static void sleep(long millis) {
		try {
			Thread.sleep(millis);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
}
