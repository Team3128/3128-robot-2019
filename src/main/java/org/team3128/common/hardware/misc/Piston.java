package org.team3128.common.hardware.misc;

import edu.wpi.first.wpilibj.Solenoid;


/**
 * Class to control a pneumatic piston, which has a two solenoids which
 * control its position.
 * @author Noah Sutton-Smolin
 */
public class Piston 
{
    private final Solenoid solA, solB;
    
    private final String logTag;
    
    private boolean isInverted = false;
   
    public Piston(int solAChannel, int solBChannel)
    {
        this(solAChannel, solBChannel, false, false);
    }
   
    public Piston(int solAChannel, int solBChannel, boolean solStateA, boolean solStateB)
    {
        this.solA = new Solenoid(solAChannel); 
        this.solB = new Solenoid(solBChannel);
        
        solA.set(solStateA); 
        solB.set(solStateB);
        
        logTag = String.format("Piston(%d, %d)", solAChannel, solBChannel);
    }
   
    /**
     * Causes the piston to switch its high and low states.
     * 
     * Also calls setPistonInvert()
     */
    public void invertPiston() 
    {
    	this.isInverted = !isInverted;
    	setPistonInvert();
    }
   
    public void lockPiston()
    {
        solA.set(true);
        solB.set(true);
        //Log.debug(logTag," set to locked state");
    }

    public void unlockPiston() 
    {
        solA.set(false);
        solB.set(false);
        //Log.debug(logTag, " set to unlocked state");
    }
   
    public void setPistonOn()
    {
        solA.set(true ^ this.isInverted);
        solB.set(false ^ this.isInverted);
        //Log.debug(logTag, " set to on state");
    }
   
    public void setPistonOff() 
    {
        solA.set(false ^ this.isInverted);
        solB.set(true ^ this.isInverted);
        //Log.debug(logTag, " set to off state");
    }
   
    /**
     * Set the piston's position to the opposite of what it currently is
     */
    public void setPistonInvert()
    {
        solA.set(!solA.get());
        solB.set(!solB.get());
        //Log.debug(logTag, " set to flip-state " + solA.get() + ", " +solB.get());
    }

}