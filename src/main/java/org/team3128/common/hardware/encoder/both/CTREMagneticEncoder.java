package org.team3128.common.hardware.encoder.both;

import org.team3128.common.util.units.Angle;

import edu.wpi.first.wpilibj.Counter;

/*        _
 *       / \ 
 *      / _ \
 *     / | | \
 *    /  |_|  \
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
 * Driver for a CTRE Magnetic Encoder using DIO ports on the roborio.
 * 
 * When instantiated, it sets the distance from the absolute angle.
 * So, between 0 and 1 rotations. When reset, the distance goes to zero.
 * 
 * Internally, it uses a Counter to measure the PWM and a WPILib Encoder object
 * to measure the quadrature part.
 * 
 * @author Narwhal
 *
 */
public class CTREMagneticEncoder extends QuadratureEncoder
{
	// had to get this from a forum post by a CTR employee
	public static final int PULSES_PER_REVOLUTION = 1024;

	Counter pwmCounter;
	
	//initial offset read from the absolute position measurement
	double encoderDegreeOffset;

	/**
	 * 
	 * @param dataAPort
	 *            DIO port with the "A" data line plugged in (pin 7 on the
	 *            encoder)
	 * @param dataBPort
	 *            DIO port with the "B" data line plugged in to it (pin 5 on the
	 *            encoder)
	 * @param pwmPort
	 *            DIO port connected to pin 9 on the encoder, the PWM pin
	 * 
	 * @param measureStartingPosition
	 *     If true, use the absolute PWM signal to figure out the absolute position of the encoder when the class is constructed.  Of
	 *     course, this can only be between 0 and 360 degrees, so if your encoder rotates multiple times, this is pretty much useless.
	 *           
	 * @param startingOffset
	 *     Degree offset TO add to the initial position measurement.  In other word, offset between the zero point of your encoder and
	 *     the zero point of your mechanism.  If measureStartingPosition is false this does nothing.
	 *           
	 * The PWM signal is used to get absolute position, while the quadrature inputs are used for relative position and velocity.
	 * 
	 * 
	 * @param inverted
	 *            whether or not the encoder is inverted
	 */
	public CTREMagneticEncoder(int dataAPort, int dataBPort, int pwmPort, boolean inverted, boolean measureStartingPosition, double startingOffset) 
	{
		super(dataAPort, dataBPort, PULSES_PER_REVOLUTION, inverted);
		
		if(measureStartingPosition)
		{
			pwmCounter = new Counter(pwmPort);
			pwmCounter.setSemiPeriodMode(false); //only count rising edges
			
			//wait for the counter to count
			try
			{
				Thread.sleep(1);
			}
			catch(InterruptedException e)
			{
				e.printStackTrace();
			}
			
			encoderDegreeOffset = getPWMAngle();
		}
	}

	/**
	 * Returns the absolute angle of the encoder from the PWM signal.  Each full rotation starts this angle back at 0.
	 * @return
	 */
	private double getPWMAngle()
	{
		//from 1 to 4096 us
		return ((pwmCounter.getPeriod() - 1e-6) / 4095e-6) * Angle.ROTATIONS ;
	}
	
	@Override
	public double getAngle()
	{
		return encoder.getDistance() + encoderDegreeOffset;
	}
	
	@Override 
	public void reset()
	{
		encoderDegreeOffset = 0;  //we don't need this any more
		super.reset();
	}

}
