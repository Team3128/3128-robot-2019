package org.team3128.testbench.main;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.util.units.Length;

import edu.wpi.first.wpilibj.Joystick;

public class MainTestBench extends NarwhalRobot {

    public TalonSRX boi1, boi2;
    public ListenerManager listenerLeft, listenerRight;
    public Joystick leftJoystick, rightJoystick;
    public double wheelCirc;
    public int lowGearMaxSpeed;

    public SRXTankDrive drive;

    @Override
    protected void constructHardware() {

        boi1 = new TalonSRX(1);
        boi2 = new TalonSRX(2);

        SRXTankDrive.initialize(boi1, boi2, wheelCirc, 1, 25.25 * Length.in, 30.5 * Length.in,
				lowGearMaxSpeed);
        drive = SRXTankDrive.getInstance();
        
        leftJoystick = new Joystick(0);
        rightJoystick = new Joystick(1);

        listenerLeft = new ListenerManager(leftJoystick);
        listenerRight = new ListenerManager(rightJoystick);

        addListenerManager(listenerLeft);
        addListenerManager(listenerRight);

    }

    @Override
    protected void setupListeners() {

        listenerRight.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
		listenerRight.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
		listenerRight.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");

		listenerRight.addMultiListener(() ->
		{
			double x = listenerRight.getAxis("MoveForwards");
			double y = listenerRight.getAxis("MoveTurn");
			double t = listenerRight.getAxis("Throttle") * -1;
			drive.arcadeDrive(x, y, t, true);
		}, "MoveForwards", "MoveTurn", "Throttle");
    }

    @Override
    protected void constructAutoPrograms() {

    }

    @Override
    protected void teleopInit() {

    }

    @Override
    protected void teleopPeriodic() {

    }

    @Override
    protected void autonomousInit() {
        
    }

}