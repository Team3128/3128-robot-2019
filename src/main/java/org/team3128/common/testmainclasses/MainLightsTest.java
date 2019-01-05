package org.team3128.common.testmainclasses;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.hardware.lights.LightsColor;
import org.team3128.common.hardware.lights.LightsSequence;
import org.team3128.common.hardware.lights.PWMLights;
import org.team3128.common.util.Log;

public class MainLightsTest extends NarwhalRobot {

	public PWMLights lights;	
	
	public static final LightsSequence lightsRainbowSequence;
	
	static
	{
		lightsRainbowSequence = new LightsSequence();

		lightsRainbowSequence.addStep(new LightsSequence.Step(LightsColor.red, 500, false));
		lightsRainbowSequence.addStep(new LightsSequence.Step(LightsColor.orange, 500, false));
		lightsRainbowSequence.addStep(new LightsSequence.Step(LightsColor.green, 500, false));
		lightsRainbowSequence.addStep(new LightsSequence.Step(LightsColor.new8Bit(1, 0xff, 0xff), 500, false));
		lightsRainbowSequence.addStep(new LightsSequence.Step(LightsColor.blue, 500, false));
		lightsRainbowSequence.addStep(new LightsSequence.Step(LightsColor.new8Bit(0xFF, 0x7F, 0x7F), 500, true));
		
		lightsRainbowSequence.setRepeat(true);

	}

	@Override
	protected void constructHardware()
	{
		lights = new PWMLights(10, 11, 12);
		
	}

	@Override
	protected void setupListeners()
	{
		
	}

	@Override
	protected void teleopInit()
	{
		//lights.setFader(LightsColor.new4Bit(9, 0xf, 0), 32, 25);
		Log.debug("MainLightsTest", "Starting lights sequence...");
		lights.executeSequence(lightsRainbowSequence);
		//lights.setFader(LightsColor.new4Bit(0xf, 0, 0));
//		testMotor.set(1);
//		try {
//			Field centerPwmField = testMotor.getClass().getSuperclass().getSuperclass().getDeclaredField("m_centerPwm");
//			centerPwmField.setAccessible(true);
//			Log.debug("MainLightsTest", Integer.toString(centerPwmField.getInt(testMotor)));
//
//		} 
//		catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e)
//		{
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}		
	}

	@Override
	protected void autonomousInit()
	{
	}

}
