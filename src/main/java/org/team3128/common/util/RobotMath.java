package org.team3128.common.util;

import org.team3128.common.util.enums.MotorDir;
import org.team3128.common.util.units.Angle;

/**
 *
 * @author Noah Sutton-Smolin
 * 
 *         Reorganized and optimized! Not original code, but same api...
 *         -morebytes
 */
public class RobotMath {
	/**
	 * Limits the angle to between 0 and 359 degrees for all math. All angles should
	 * be normalized before use.
	 * <p/>
	 * 
	 * @param angle the angle to be normalized
	 *              <p/>
	 * @return the normalized angle on [0, 359]
	 */
	public static double normalizeAngle(double angle) {
		return ((angle % 360) + 360) % 360;
	}

	// TODO: many of these functions exist in Java's Math class now that we aren't
	// using Java ME
	/**
	 * Finds the shortest distance between two angles.
	 *
	 * @param angle1   angle
	 * @param angle2   angle
	 * @param shortWay if true, go the shorter way to make moves always <= 180
	 * @return shortest angular distance between
	 */
	public static double angleDistance(double angle1, double angle2, boolean shortWay) {
		double retval = normalizeAngle(angle2) - normalizeAngle(angle1);

		if (shortWay && Math.abs(retval) > 180) {
			retval = -(RobotMath.sgn(retval)) * (360 - Math.abs(retval));
		}

		return retval;
	}

	/**
	 * Standard-ish sign function
	 * 
	 * @param n
	 * @return
	 */
	public static double sgn(double n) {
		if (n != 0) {
			n = Math.abs(n) / n;
		}

		return n;

	}

	public static int sgn(int n) {
		if (n != 0) {
			n = Math.abs(n) / n;
		}

		return n;
	}

	/**
	 * Determines the appropriate direction for a motor to turn to get to an angle.
	 * <p/>
	 * 
	 * @param currentAngle the current angle of the motor
	 * @param targetAngle  the target angle of the motor
	 * @param shortWay     if true, go the shorter way to make moves always <= 180
	 *                     <p/>
	 * @return a MotorDir
	 */
	public static MotorDir getMotorDirToTarget(double currentAngle, double targetAngle, boolean shortWay) {
		MotorDir retval;

		currentAngle = RobotMath.normalizeAngle(currentAngle);
		targetAngle = RobotMath.normalizeAngle(targetAngle);
		int difference = ((shortWay && Math.abs(currentAngle - targetAngle) > 180) ? 1 : -1)
				* (currentAngle - targetAngle < 0 ? -1 : 1);

		if (Math.abs(currentAngle - targetAngle) < .001)
			retval = MotorDir.NONE;
		else
			retval = (difference == 1) ? MotorDir.CW : MotorDir.CCW;

		return retval;
	}

	/**
	 * Clamps value from (inclusive) minimum to maximum
	 * 
	 * @param value
	 * @param minimum
	 * @param maximum
	 * @return
	 */
	public static int clamp(int value, int minimum, int maximum) {
		if (!(minimum <= maximum)) {
			Log.unusual("RobotMath", "...what?  clampInt() called with insane arguments");
		}
		return Math.min(Math.max(value, minimum), maximum);
	}

	/**
	 * Clamps value from (inclusive) minimum to maximum
	 * 
	 * @param value
	 * @param minimum
	 * @param maximum
	 * @return
	 */
	public static double clamp(double value, double minimum, double maximum) {
		return Math.min(Math.max(value, minimum), maximum);
	}

	/**
	 * Clamps value from (inclusive) minimum to maximum (template function version)
	 * 
	 * @param value
	 * @param minimum
	 * @param maximum
	 * @return
	 */
	public static <T extends Comparable<T>> T clamp(T value, T minimum, T maximum) {
		if (!(minimum.compareTo(maximum) < 0)) {
			Log.unusual("RobotMath", "...what?  clampInt() called with insane arguments");
			return value;
		}

		if (minimum.compareTo(value) > 0) {
			return minimum;
		} else if (maximum.compareTo(value) < 0) {
			return maximum;
		} else {
			return value;
		}
	}

	/**
	 * Clamps value between positive and negative 1 and returns value.
	 * 
	 * Great for motor powers!
	 * 
	 * @param d
	 */
	public static double clampPosNeg1(double d) {
		d = clamp(d, -1, 1);
		return d;
	}

	public static final double SQUARE_ROOT_TWO = Math.sqrt(2.0);

	/**
	 * If the abs value of the number is less than the threshold, return 0,
	 * otherwise return the number
	 * 
	 * @param value
	 * @param threshold
	 * @return
	 */
	public static double thresh(double value, double threshold) {
		double retval;

		if (Math.abs(value) < Math.abs(threshold)) {
			retval = 0;
		} else {
			retval = value;
		}

		return retval;
	}

	/**
	 * Converts angular distance to linear.
	 * 
	 * For example, if a wheel was touching a surface, and the wheel turned y
	 * degrees, then the surface moved AngularDistToLinear(y) degrees.
	 * 
	 * @param cm
	 * @param wheelCircumference the circumference of the circle
	 * @return
	 */
	public static double angularDistToLinear(double deg, double circumference) {
		return (deg / 360) * circumference;
	}

	// hidden constructor
	private RobotMath() {
	}

	/**
	 * Squares the argument. Easier than Math.pow(number, 2).
	 * 
	 * @param number
	 * @return
	 */
	public static double square(double number) {
		return number * number;
	}

	/**
	 * Squares the argument. Easier than RobotMath.floor_double_int(Math.pow(number,
	 * 2)).
	 * 
	 * @param number
	 * @return
	 */
	public static int square(int number) {
		return number * number;
	}

	/**
	 * Returns the exponent needed to make e (Euler's number) the given number.
	 * 
	 * @param d
	 */
	public static double logE(double d) {
		return Math.log(d);
	}

	/**
	 * Returns the exponent needed to make 10 the given number.
	 * 
	 * @param d
	 */
	public static double log10(double d) {
		return Math.log10(d);
	}

	// calculate this once here for speed
	static final private double log2Base10 = Math.log10(2);

	/**
	 * Returns the exponent needed to make 2 the given number.
	 * 
	 * @param d
	 */
	public static double log2(double d) {
		return Math.log10(d) / log2Base10;
	}

	/**
	 * Returns the exponent needed to make an arbitrary base the given number.
	 * 
	 * @param d
	 */
	public static double logN(double base, double num) {
		return Math.log(num) / Math.log(base);
	}

	/**
	 * Raises an integer to a integer power >= 0.
	 * 
	 * Much more lightweight than the regular function, but also more restricted.
	 * Negative powers are treated as 0.
	 */
	public static int intPow(int number, int power) {
		int retval = 1;

		for (; power >= 1; --power) {
			retval *= number;
		}

		return retval;
	}

	/**
	 * Raises a double to a integer power >= 0.
	 * 
	 * More lightweight than the regular function, but also more restricted.
	 * Negative powers are treated as 0.
	 */
	public static double intPow(double number, int power) {
		double retval = 1;

		for (; power >= 1; --power) {
			retval *= number;
		}

		return retval;
	}

	/**
	 * Returns d rounded to the nearest integer (ties round towards positive
	 * infinity).
	 * 
	 * Throws if the argument can't fit in an int.
	 * 
	 * @param d
	 */
	public static int round(double d) {
		d = Math.round(d);

		if (Math.abs(d) > Integer.MAX_VALUE) {
			throw new IllegalArgumentException("Provided number is too large to be an integer!");
		}

		return (int) d;
	}

	/**
	 * Returns the cosine of angle d in degrees.
	 * 
	 * @param d The angle is degrees.
	 */
	public static double cos(double d) {
		return Math.cos(Math.toRadians(d)); // convert to radians
	}

	/**
	 * Returns the sine of angle d in degrees.
	 * 
	 * @param d The angle is degrees.
	 */
	public static double sin(double d) {
		return Math.sin(Math.toRadians(d));
	}

	/**
	 * Returns the tangent of angle d in degrees.
	 * 
	 * @param d The angle is degrees.
	 */
	public static double tan(double d) {
		return Math.tan(Math.toRadians(d));
	}

	/**
	 * Returns the smallest integer greater to or equal to d.
	 * 
	 * Throws if the result cannot be represented by an integer.
	 * 
	 * @param d
	 */
	public static int ceil(double d) {
		d = Math.ceil(d);

		if (!Double.isFinite(d) || d > Integer.MAX_VALUE) {
			throw new IllegalArgumentException("The provided double cannot be represented by an int");
		}

		return (int) d;
	}

	/**
	 * Returns the largest integer smaller to or equal to d.
	 * 
	 * Throws if the result cannot be represented by an integer.
	 * 
	 * @param d
	 */
	public static int floor(double d) {
		d = Math.floor(d);

		if (!Double.isFinite(d) || d > Integer.MAX_VALUE) {
			throw new IllegalArgumentException("The provided double cannot be represented by an int");
		}

		return (int) d;
	}

	/**
	 * Returns the arc-sine of d - the angle in degrees whose sine is d.
	 * 
	 * @param d
	 */
	public static double asin(double d) {
		return Math.asin(d) * Angle.RADIANS;
	}

	/**
	 * Returns the arc-cosine of d - the angle in degrees whose cosine is d.
	 * 
	 * @param d
	 */
	public static double acos(double d) {
		return Math.acos(d) * Angle.RADIANS;
	}

	/**
	 * Returns the arc-tangent of d - the angle in degrees whose tangent is d.
	 * 
	 * @param d
	 */
	public static double atan(double d) {
		return Math.atan(d) * Angle.RADIANS;
	}

	/**
	 * Returns the angle in degrees whose Tan is y/x.
	 * 
	 * Takes the X and Y values separately so that the result can be placed in the
	 * correct quadrant.
	 * 
	 * @param x
	 * @param y
	 */
	public static double atan2(double x, double y) {
		return Math.atan2(y, x) * Angle.RADIANS;
	}

	/**
	 * Evaluates the polynomial in the form f(x) = a_{n-1} * x^{n-1} + a_{n-2} *
	 * x^{n-2} + ... + a_1 * x^1 + a_0 * x^0
	 * 
	 * @param x
	 * @param coefficients - The array of coefficients, in the form {a_(n-1),
	 *                     a_(n-2), ... , a_(1), a_(0)}
	 * 
	 * @return f(x)
	 */
	public static double polynomial(double x, double... coefficients) {
		double result = 0;

		int n = coefficients.length;

		for (int pow = 0; pow < coefficients.length; pow += 1) {
			result += coefficients[n - pow - 1] * Math.pow(x, pow);
		}

		return result;
	}

	/**
	 * Calculates the linear distance between two points (x0, y0) and (x1, y1)
	 * specified in cartesian coordinates, using Pythagorean Theroem.
	 * 
	 * @param x0 - The x-coordinate of the first point
	 * @param y0 - The y-coordinate of the first point
	 * @param x1 - The x-coordinate of the second point
	 * @param y1 - The y-coordinate of the second point
	 * 
	 * @return the distance between (x0, y0) and (x1, y1)
	 */
	public static double distance(double x0, double y0, double x1, double y1) {
		return Math.sqrt(Math.pow(x0 - x1, 2) + Math.pow(y0 - y1, 2));
	}
}
