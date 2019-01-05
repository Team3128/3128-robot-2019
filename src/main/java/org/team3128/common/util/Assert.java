package org.team3128.common.util;

import org.team3128.common.NarwhalRobot;

import edu.wpi.first.wpilibj.DriverStation;

public class Assert
{
	private static NarwhalRobot robot;
	
	public static void setRobot(NarwhalRobot newRobot)
	{
		robot = newRobot;
	}
	
	/**
	 * Generic assert on a boolean.
	 * 
	 * Try to use one of the more specific functions below if possible so that a more detailed error message can be printed.
	 * @param condition
	 */
	public static void that(boolean condition)
	{
		if(!condition)
		{
			assertFail("Condition should be true, but is false!");
		}
	}
	
	public static void that(boolean condition, String failureMessage)
	{
		if(!condition)
		{
			assertFail(failureMessage);
		}
	}
	
	/**
	 * Call this to kill the robot and print the given message as the cause
	 */
	public static void fail(String message)
	{
		assertFail(message);
	}
	
	public static void isNull(Object object)
	{
		if(object != null)
		{
			assertFail(String.format("Should be null, but is actually \"%s\"!", object.toString()));
		}
	}
	
	public static void notNull(Object object)
	{
		if(object == null)
		{
			assertFail("Should not be null!");
		}
	}
	
	public static void equal(int integer1, int integer2)
	{
		if(integer1 != integer2)
		{
			assertFail(String.format("Should be equal, but first argument is %d and second is %d", integer1, integer2));
		}
	}
	
	/**
	 * Assert that the argument is positive or 0.
	 */
	public static void positive(long number)
	{
		if(number < 0)
		{
			assertFail(String.format("Should be positive, but is actually %d", number));
		}
	}
	
	/**
	 * Assert that the argument is positive or 0.
	 */
	public static void positive(double number)
	{
		if(number < 0)
		{
			assertFail(String.format("Should be positive, but is actually %d", number));
		}
	}
	
	/**
	 * Assert that the argument is negative.  0 is NOT a valid value.
	 */
	public static void negative(long number)
	{
		if(number >= 0)
		{
			assertFail(String.format("Should be negative, but is actually %d", number));
		}
	}
	
	/**
	 * Assert that the argument is negative.  0 is NOT a valid value.
	 */
	public static void negative(double number)
	{
		if(number >= 0)
		{
			assertFail(String.format("Should be negative, but is actually %d", number));
		}
	}
	
	/**
	 * Assert that the number is in the supplied range.  Inclusive.
	 * @param number
	 * @param min
	 * @param max
	 */
	public static void inRange(int number, int min, int max)
	{
		if(min > max)
		{
			assertFail("Assert argument error: Minumum is greater than maximum");
		}
		
		if(number < min || number > max)
		{
			assertFail(String.format("Should be between %d and %d, but actually is %d!", min, max, number));
		}
	}
	
	public static void inRange(double number, double min, double max)
	{
		if(min > max)
		{
			assertFail("Assert argument error: Minumum is greater than maximum");
		}
		
		if(number < min || number > max)
		{
			assertFail(String.format("Should be between %.03f and %.03f, but actually is %.03f!", min, max, number));
		}
	}
	
	public static void validMotorPower(double power)
	{
		if(power < -1 || power > 1)
		{
			assertFail(String.format("%.03f is not a valid motor power", power));
		}
	}
	
	public static void equals(Object obj1, Object obj2)
	{	
		if(!obj1.equals(obj2))
		{
			assertFail(String.format("%s is not equal to %s!", obj1 == null ? "null" : obj1.toString(), obj2 == null ? "null" : obj2.toString()));
		}
	}
	
	public static void equals(int num1, int num2)
	{	
		if(num1 != num2)
		{
			assertFail(String.format("%d is not equal to %d!", num1, num2));
		}
	}
	
	public static void greaterThan(double number, double lowerBound)
	{
		if(number <= lowerBound)
		{
			assertFail(String.format("%.03f is not greater than %.03f!", number, lowerBound));
		}
	}
	
	public static void greaterThan(int number, int lowerBound)
	{
		if(number <= lowerBound)
		{
			assertFail(String.format("%d is not greater than %d!", number, lowerBound));
		}
	}
	
	public static <T extends Comparable<T>> void greaterThan(T object, T lowerBound)
	{
		if(object.compareTo(lowerBound) <= 0)
		{
			assertFail(String.format("%s is not greater than %s!", object == null ? "null" : object.toString(), lowerBound == null ? "null" : lowerBound.toString()));
		}
	}
	
	public static <T extends Comparable<T>> void lessThan(T object, T upperBound)
	{
		if(object.compareTo(upperBound) >= 0)
		{
			assertFail(String.format("%s is not less than %s!", object == null ? "null" : object.toString(), upperBound == null ? "null" : upperBound.toString()));
		}
	}
	
	public static void lessThan(double number, double upperBound)
	{
		if(number >= upperBound)
		{
			assertFail(String.format("%.03f is not less than %.03f!", number, upperBound));
		}
	}
	
	public static void lessThan(int number, int upperBound)
	{
		if(number >= upperBound)
		{
			assertFail(String.format("%d is not less than %d!", number, upperBound));
		}
	}
	
	private static void assertFail(String message)
	{
		robot.zeroOutListeners();
		
		StringBuilder failureMessage = new StringBuilder();
		
		failureMessage.append("\n\n--------------------------------------------------------------\n");
		failureMessage.append("ASSERT FAILED\n\n");
		failureMessage.append(message);
		failureMessage.append("\n\nBacktrace follows: \n");
		
		StackTraceElement[] stackTrace = Thread.currentThread().getStackTrace();
		
		//start counting at 2 to start at the function where the assert failed
		for(int index = 2; index < stackTrace.length; ++index)
		{
			failureMessage.append(String.format(
					"at %s.%s() (%s:%d)\n",
					stackTrace[index].getClassName(),
					stackTrace[index].getMethodName(),
					stackTrace[index].getFileName(),
					stackTrace[index].getLineNumber()));
		}
		
		failureMessage.append("\nExiting robot code");
		failureMessage.append("--------------------------------------------------------------");

		System.err.println(message);
		System.err.flush();
		
		DriverStation.reportError(message, false);
		
		try
		{
			Thread.sleep(200);
		}
		catch(InterruptedException e)
		{
			e.printStackTrace();
		}
		
		// No more Watchdog.killRobot(), so we have to improvise
		System.exit(3128);

	}
	

}
