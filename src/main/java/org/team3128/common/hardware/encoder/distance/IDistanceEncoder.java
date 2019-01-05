package org.team3128.common.hardware.encoder.distance;

/**
 * Interface to represent a generic angular encoder.
 * @author Jamie
 *
 */
public interface IDistanceEncoder
{
	/**
	 * Returns angle in degrees
	 * @return
	 */
    public double getAngle();
    
    /**
     * Returns the raw encoder value from the hardware.
     * @return
     */
    public double getRawValue();
    
    /**
     * 
     * @return true if the encoder can revolve multiple times in the same direction, 
     * e.g. a magnetic encoder
     */
    public boolean canRevolveMultipleTimes();
    
    /**
     * Reset the encoder value to be 0 at the current position.
     */
    public void reset();
}

