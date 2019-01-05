package org.team3128.common.testmainclasses;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.hardware.ultrasonic.IUltrasonic;
import org.team3128.common.hardware.ultrasonic.MaxSonar;
import org.team3128.common.hardware.ultrasonic.MaxSonar.Resolution;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controllers.ControllerXbox;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class MainUltrasonicTest extends NarwhalRobot {

	public ListenerManager lm;

	public IUltrasonic testUltrasonic;

	@Override
	protected void constructHardware()
	{
		lm = new ListenerManager(new Joystick(4));

		testUltrasonic = new MaxSonar(2, Resolution.MM, Port.kOnboard);		
	}

	@Override
	protected void setupListeners()
	{
		addListenerManager(lm);
		
		lm.nameControl(ControllerXbox.X, "Ping");

		lm.nameControl(ControllerXbox.LB, "AutoPingOn");
		lm.nameControl(ControllerXbox.RB, "AutoPingOff");

		lm.addButtonDownListener("Ping", () -> System.out.println(testUltrasonic.getDistance()));
		lm.addButtonDownListener("AutoPingOff", () -> testUltrasonic.setAutoPing(false));
		
		lm.addButtonDownListener("AutoPingOn", () -> testUltrasonic.setAutoPing(true));
		
	}

	@Override
	protected void teleopInit()
	{
		
	}

	@Override
	protected void autonomousInit()
	{
		
	}

}
