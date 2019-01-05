package org.team3128.common.hardware.motor.logic;

import org.team3128.common.hardware.encoder.velocity.IVelocityEncoder;
import org.team3128.common.hardware.motor.MotorLogic;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.VelocityPID;
import org.team3128.common.util.datatypes.PIDConstants;

/*        _
 *       / \ 
 *      / _ \
 *     / [ ] \
 *    /  [_]  \
 *   /    _    \
 *  /    (_)    \
 * /_____________\
 * -----------------------------------------------------
 * UNTESTED CODE!
 * This class has never been tried on an actual robot.
 * It may be non or partially functional.
 * Do not make any assumptions as to its behavior!
 * And don't blink.  Not even for a second.
 * -----------------------------------------------------*/
/**
 * Speed control that uses PID to hit its target speed more accurately.
 *
 * @author Jamie
 */
public class PIDSpeedLogic extends MotorLogic
{   
    private IVelocityEncoder _encoder;
    
    private VelocityPID _pidCalculator;
    
    /**
     * Target speed in RPM.
     */
    protected double _targetSpeed;
    

    /**
     *
     * @param tgtSpeed    target speed in rpm
     * @param refreshTime speed update rate in msec
     * @param encoder the encoder to use
     * @param kP the Konstant of Proportional
     * @param kI the Konstant of Integral
     * @param kD the Konstant of Derivative
     */
    public PIDSpeedLogic(double tgtSpeed, int refreshTime, IVelocityEncoder encoder, PIDConstants pidConstants)
    {
    	_targetSpeed = tgtSpeed;
        _refreshTime = refreshTime;
        _encoder = encoder;
        _pidCalculator = new VelocityPID(pidConstants);
        _pidCalculator.setDesiredVelocity(tgtSpeed);
    }
   
    /**
     * Uses a default refreshTime of 50msec
     *
     * @param tgtSpeed target speed in rpm
     */
    public PIDSpeedLogic(double tgtSpeed, IVelocityEncoder encoder, VelocityPID pidCalc)
    {
        _encoder = encoder;
        _pidCalculator = pidCalc;
    }
   
    @Override
    public synchronized void setControlTarget(double d)
    {
        _pidCalculator.resetIntegral();
        _targetSpeed = d;
    	_pidCalculator.setDesiredVelocity(d);
    }

    @Override
    public double speedControlStep(double dt)
    {
    	double speed = -1 * _encoder.getAngularSpeed();
    	if(Math.abs(speed) < 5.0)
    	{
    		speed = 0;
    	}
        _pidCalculator.update(speed);
        
        double output = RobotMath.clampPosNeg1(_pidCalculator.getOutput());
        
        Log.debug("PIDSpeedLogic", "Current RPM: " + speed + " Output: " + output);
        
        
        return output;
    }
   
    @Override
    public synchronized void reset()
    {
        setControlTarget(0);
        _pidCalculator.resetIntegral();
    }

    @Override
    public boolean isComplete()
    {
    	return false;
    }
}
