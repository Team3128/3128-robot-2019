package org.team3128.common.hardware.encoder.distance;

import org.team3128.common.util.units.Angle;

import edu.wpi.first.wpilibj.AnalogInput;


/**
 * Interfaces with magnetic angular encoders,
 * like the ones we used in the 2014 season. 
 *
 */
public class MagneticPotentiometerEncoder implements IDistanceEncoder
{
    private AnalogInput enc;
    private double offset;
   
    public MagneticPotentiometerEncoder(int port)
    {
        enc = new AnalogInput(port);
        this.offset = 0;
    }
   
    public MagneticPotentiometerEncoder(double offset, int port)
    {
        enc = new AnalogInput(port);
        this.offset = offset;
    }
   
    /**
     * Gets the approximated angle from a magnetic encoder. It uses values which
     * have been estimated to high accuracy from extensive tests. Unless need be, 
     * do not modify these values.
     *
     * @return the approximate angle from 0 to 360 degrees of the encoder
     */
    @Override
    public double getAngle() 
    {
        double voltage = 0;
        
        for(char i = 0; i<10; i++)
        {
            voltage += enc.getVoltage();
        }
        
        voltage /= 10;
        return (voltage/5.0*Angle.ROTATIONS)+offset;
    }

    /**
     *
     * @return the raw voltage of the encoder
     */
    public double getRawValue() {return enc.getVoltage();}

	@Override
	public boolean canRevolveMultipleTimes()
	{
		return true;
	}

	@Override
	public void reset()
	{
		offset = getAngle();
	}
}

