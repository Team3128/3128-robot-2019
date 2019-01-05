package org.team3128.common.testmainclasses;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controllers.ControllerXbox;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;

public class MainPneumaticsTest extends NarwhalRobot
{
	
	public ListenerManager lmExtreme;
	
	public Piston testPiston;
	
	public Compressor compressor;
	
	@Override
	protected void constructHardware()
	{
		lmExtreme = new ListenerManager(new Joystick(0));	
		
		testPiston = new Piston(0, 1);
		testPiston.invertPiston();
		
		compressor = new Compressor();
		compressor.setClosedLoopControl(true);
	}

	@Override
	protected void setupListeners()
	{
		addListenerManager(lmExtreme);
		
		testPiston.setPistonOff();
		
		lmExtreme.nameControl(ControllerXbox.A, "PistonExtend");
		lmExtreme.nameControl(ControllerXbox.B, "PistonRetract");

		lmExtreme.nameControl(ControllerXbox.LB, "CompressorOn");
		lmExtreme.nameControl(ControllerXbox.RB, "CompressorOff");

		
		testPiston.unlockPiston();
	
		
		lmExtreme.addButtonDownListener("PistonExtend", () -> testPiston.setPistonOn());

		lmExtreme.addButtonDownListener("CompressorOn", () -> compressor.start());
		lmExtreme.addButtonDownListener("CompressorOff", () -> compressor.stop());		
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
