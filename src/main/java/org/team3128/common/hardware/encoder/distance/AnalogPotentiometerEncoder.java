package org.team3128.common.hardware.encoder.distance;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * Represents a variable resistor/potentiometer encoder. 
 * @author Kian, Jamie
 *
 */
public class AnalogPotentiometerEncoder extends SendableBase implements IDistanceEncoder, Sendable
{
	private AnalogInput enc;
	private double degreesPerVolt;
    private double offset;
    
    private NetworkTable table;
    
    /**
     * 
     * @param chan
     * @param off offset in degrees
     * @param voltsAtEndOfTravel the voltage when the encoder is at the end of its travel
     * @param travelLength the length in degrees of the travel
     */
    public AnalogPotentiometerEncoder(int chan, int off, double voltsAtEndOfTravel, double travelLength)
    {
		enc = new AnalogInput(chan);
		offset = off;
		
		degreesPerVolt = travelLength / voltsAtEndOfTravel;
	}
	
	@Override
	public double getAngle() {
		
		return (getRawValue() * degreesPerVolt) + offset;
	}

	@Override
	public double getRawValue()
	{
		return enc.getVoltage();
	}

	@Override
	public boolean canRevolveMultipleTimes()
	{
		return false;
	}


	@Override
	public void reset()
	{
		offset = getAngle();
		
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Potentiometer");
		
		builder.addDoubleProperty("Angle", () -> {
			return getAngle();
		}, null);
	}

}