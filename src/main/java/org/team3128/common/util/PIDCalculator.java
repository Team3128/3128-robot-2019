package org.team3128.common.util;

import org.team3128.common.util.datatypes.PIDConstants;
import org.team3128.common.util.datatypes.Pair;
import org.team3128.common.util.datatypes.RandomAccessBuffer;

/**
 * Class to calculate positional PID.
 * 
 * Handles integration zone and compensates for irregular update times.
 * @author Jamie
 *
 */
public class PIDCalculator
{
	// if the time between updates exceeds this value, the integral will be reset
	private final double INTEGRATION_TIMEOUT = 1000; //ms
	
	private PIDConstants constants;
	
	// buffer containing the time duration that the feedback was a value, and the value
	private RandomAccessBuffer<Pair<Integer, Double>> pastValues;
	
	private double previousError = 0;
	private long previousValueTime;
	
	private double target;
	
	private double threshold;
	private int numCyclesInThreshold = 0;
	
	private boolean log = false;
	private String logName;
	
	/**
	 * @param constants
	 * @param izone the number of past measurements to keep when calculating the I term
	 * @param threshold the "acceptable" amount of error inside which the value is considered close enough and the threshold count will be incremented
	 */
	public PIDCalculator(PIDConstants constants, int izone, double threshold)
	{
		this.constants = constants;
		pastValues = new RandomAccessBuffer<>(izone);
		previousValueTime = System.currentTimeMillis();
		
		this.threshold = threshold;
	}
	
	public void setConstants(PIDConstants constants)
	{
		this.constants = constants;
	}
	
	/**
	 * Zero out the accumulated I value
	 */
	public void resetIntegral()
	{
		for(int index= 0; index <= pastValues.getLastIndex(); ++index)
		{
			pastValues.set(index, null);
		}
	}
	
	public void setTarget(double target)
	{
		this.target = target;
	}
	
	/**
	 * Calculate the trapezoidal sum of the integration data.
	 * @return
	 */
	private double calculateIntegral()
	{
		if(pastValues.getSize() <= 1)
		{
			return 0;
		}
		
		//we use trapezoidal integration
		
		double integral = 0;
		for(int index = 1; index <= pastValues.getLastIndex(); ++index)
		{
			integral += ((pastValues.get(index - 1).right + pastValues.get(index).right)) * pastValues.get(index-1).left;
		}
		integral /= 2;
		
		return integral;
	}
	
	public double update(double value)
	{
		// Cap the delta time to prevent it going through the roof when the robot is disabled
		int deltaTime = (int) RobotMath.clamp(System.currentTimeMillis() - previousValueTime, 0, 100);
		
		double error = target - value;
		
		double derivative = -1 * (error - previousError) / deltaTime;
				
		// add the time segment ending now to the integral
		pastValues.enqueue(new Pair<Integer, Double>(deltaTime, error));
		
		double output = error * constants.getkP() + calculateIntegral() * constants.getkI() + derivative * constants.getkP();
		
		// move one step forward
		previousError = error;
		previousValueTime = System.currentTimeMillis();
		
		// check thresholding
		if(Math.abs(error) < threshold)
		{
			++numCyclesInThreshold;
		}
		else
		{
			numCyclesInThreshold = 0;
		}
		
		if(log)
		{
			Log.debug("PIDCalculator: " + logName, String.format("Input: %.04f, Error: %.04f, output: %.04f, integral: %.04f, derivative: %.04f", value, error, output, calculateIntegral(), derivative));
		}
		
		return output;
	}
	
	/**
	 * Get the number of update() calls in a row ending with the most recent one that the error has been inside the threshold 
	 * @return
	 */
	public int getNumUpdatesInsideThreshold()
	{
		return numCyclesInThreshold;
	}
	
	/**
	 * Causes this PIDCalculator to print its input, error, and output to the log.
	 * @param logName Prefix for log messages from this calculator.
	 */
	public void enableLogging(String logName)
	{
		this.logName = logName;
		log = true;
	}
}
