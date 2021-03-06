package org.team3128.common.hardware.motor.logic;

import org.team3128.common.hardware.encoder.distance.IDistanceEncoder;
import org.team3128.common.hardware.motor.MotorLogic;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;

/**
 * Motor control which steers the motor to an angle using an encoder.
 * @author Yousuf Soliman
 * @author Jamie
 */

public class AbsolutePIDAngleLogic extends MotorLogic
{
    private double targetAngle, threshold;
    private IDistanceEncoder _encoder;
    
    private int consecutiveCorrectPositions = 0;
    
    boolean _stopWhenDone;
    double kP;
    double kI;
    double kD;
    
    double errorSum = 0;
    double prevError = 0;
    
    boolean _log;
    
    final static double errorLimit = 100000;
    
    /**
     * 
     * @param kP constant of pid
     * @param threshold acceptable error in degrees
     * @param stopWhenDone whether to stop controlling the motor when it's reached its target
     * @param encoder
     */
    public AbsolutePIDAngleLogic(double kP, double kI, double kD, double threshold, boolean stopWhenDone, IDistanceEncoder encoder, boolean log)
    {
    	_refreshTime = 10;
        
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        
        _log = log;
        
        this.threshold = threshold;
        _encoder = encoder;
        
        _stopWhenDone = stopWhenDone;
    }

    /**
     * sets degree value to move to
     */
    @Override
    public synchronized void setControlTarget(double val)
    {
        this.targetAngle = val;
        
        //reset error
        errorSum = 0;
    }

    @Override
    public double speedControlStep(double dt)
    {
    	double angle = _encoder.getAngle();
    	
    	double error = RobotMath.angleDistance(angle, this.targetAngle, _encoder.canRevolveMultipleTimes());
    	    	
    	errorSum += error;
    	
    	if(errorSum > errorLimit)
    	{
    		Log.unusual("PIDAngleTarget", "I error sum of " + errorSum + " went over limit of " + errorLimit);
    		//errorSum = errorLimit;
    	}
    	else if(errorSum < -errorLimit)
    	{
    		Log.unusual("PIDAngleTarget", "I error sum of " + errorSum + " went under limit of " + -errorLimit);
    		//errorSum = -errorLimit;
    	}
    	
        double output = error * kP + errorSum * kI + kD * (error - prevError) / dt;
        
        prevError = error;
        
       	if(_log)
    	{
            //Log.debug("PIDAngleTarget", "target: " + targetAngle + " current: " + angle + " error: " + error + " output: " + output);
    	}

        if(Math.abs(error) < threshold)
        {
        	++consecutiveCorrectPositions;
        	return 0;
        }
        consecutiveCorrectPositions = 0;
        
        return RobotMath.clampPosNeg1(output);
    }

    @Override
    public synchronized void reset()
    {
    	errorSum = 0;
    	consecutiveCorrectPositions = 0;
    	
    	targetAngle = 0;
    }

    /**
     * Returns true if the motor is at the correct angle
     */
    public boolean isComplete()
    {	
        return _stopWhenDone && consecutiveCorrectPositions >= 5;
    }
    
}

