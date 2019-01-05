package org.team3128.common.hardware.encoder.both;

import org.team3128.common.hardware.encoder.distance.IDistanceEncoder;
import org.team3128.common.hardware.encoder.velocity.IVelocityEncoder;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;

/**
 * Class that uses a quadrature encoder via WPIlib and the FPGA to measure speed.
 * @author Narwhal
 *
 */
public class QuadratureEncoder implements IVelocityEncoder, IDistanceEncoder
{
	Encoder encoder;
	
	/**
	 * 
	 * @param dataAPort DIO port with the "A" data line plugged in to it
	 * @param dataBPort DIO port with the "B" data line plugged in to it
	 * @param pulsesPerRevolution The pulses per revolution of the encoder.  It should say on the encoder
	 * or its datasheet.
	 * @param inverted whether or not the encoder is inverted
	 */
	public QuadratureEncoder(int dataAPort, int dataBPort, double pulsesPerRevolution, boolean inverted) 
	{
		encoder = new Encoder(dataAPort, dataBPort, inverted, EncodingType.k4X);
		encoder.setDistancePerPulse(360/pulsesPerRevolution);
	}

	@Override
	public double getAngularSpeed() {
		
		return encoder.getRate();
	}


	@Override
	public double getAngle()
	{
		return encoder.getDistance();
	}

	@Override
	public double getRawValue()
	{
		return encoder.get();
	}

	@Override
	public boolean canRevolveMultipleTimes()
	{
		//I THINK this is true of all quadrature encoders...
		return true;
	}

	@Override
	public void reset()
	{
		encoder.reset();
	}

}
