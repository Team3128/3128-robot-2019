package org.team3128.gromit.main;

import com.ctre.phoenix.motorcontrol.InvertType;

import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.util.units.Angle;
import org.team3128.common.util.units.Length;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;

public class MainWallace extends MainDeepSpaceRobot {
    Piston placeholder1, placeholder2;    
    

    @Override
    protected void constructHardware() {
        wheelbase = 37 * Length.in;
        driveMaxSpeed = 5800;
        //gearRatio = 2.9 + 54/990;
        wheelCirc = 12.01 * Length.in;

        leftSpeedScalar = 1.00;
        rightSpeedScalar = 1.00;

        // TODO: 6ft/s, 7ft/s
        shiftUpSpeed = 100000;
        shiftDownSpeed = -1;
    
        //26
        gearshiftPiston = new Piston(5, 2);
        gearshiftPiston.setPistonOn();

        placeholder1 = new Piston(1, 0);
        placeholder1.setPistonOff();

        demogorgonPiston = new Piston(3, 4);

        placeholder2 = new Piston(7, 6);
        placeholder2.setPistonOn();

        liftLimitSwitch = new DigitalInput(0);
        liftSwitchPosition = 110;
        liftMaxVelocity = 4200;

        fourBarLimitSwitch = new DigitalInput(1);
        fourBarRatio = 4600 / (180 * Angle.DEGREES);
        fourBarSwitchPosition = +99 * Angle.DEGREES;
        fourBarMaxVelocity = 100;
        
        cargoBumperSwitch = new DigitalInput(2);
        
        super.constructHardware();

        leftDriveLeader.setInverted(true);
        leftDriveFollower.setInverted(InvertType.FollowMaster);

        leftDriveLeader.setSensorPhase(true);

        rightDriveLeader.setInverted(true);
        rightDriveFollower.setInverted(InvertType.FollowMaster);

        rightDriveLeader.setSensorPhase(true);

        // Lift Inverts
        liftMotorLeader.setInverted(false);
        liftMotorFollower.setInverted(false);

        liftMotorLeader.setSensorPhase(true);


        // Lift Intake Invert
        liftIntakeMotor.setInverted(false);

        // FourBar Invert
        fourBarMotor.setInverted(false);
        fourBarMotor.setSensorPhase(false);

        //2 is big camera for lars KEEP AT 2
        limelight.driverMode(2);
        limelight.turnOffLED();
    }
    /*
    @Override
    protected void setupListeners() {
        // REGULAR CONTROLS
        //super.setupListeners();
		// Drive
        listenerRight.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
		listenerRight.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
		listenerRight.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");
        
        listenerRight.addMultiListener(() ->
		{
			if (!runningCommand) {
				double vert =     -1.0 * listenerRight.getAxis("MoveForwards");
				//DEBUG: IF ANYTHING GOES WRONG, CHANGE TO -0.8 (ADHAM AND JUDE REMEMBER)
				double horiz =    -0.8 * listenerRight.getAxis("MoveTurn");
				double throttle = -1.0 * listenerRight.getAxis("Throttle");
	
                drive.arcadeDrive(vert, horiz, throttle, true);
                //Log.info("Joystick", "IT WORKS");
			}
        }, "MoveForwards", "MoveTurn", "Throttle");
        
        listenerLeft.nameControl(ControllerExtreme3D.TRIGGER, "Override");
		listenerLeft.nameControl(new Button(2), "ManualMode");
		listenerLeft.nameControl(ControllerExtreme3D.JOYY, "ManualControl");

		listenerLeft.addMultiListener(() -> {
			if (listenerLeft.getButton("ManualMode")) {
				lift.override = false;
				lift.powerControl(0);

				fourBar.override = listenerLeft.getButton("Override");
				fourBar.powerControl(listenerLeft.getAxis("ManualControl"));
			}
			else {
				fourBar.override = false;
				fourBar.powerControl(0);

				lift.override = listenerLeft.getButton("Override");
				lift.powerControl(listenerLeft.getAxis("ManualControl"));
			}
        }, "ManualMode", "Override", "ManualControl");
        listenerLeft.nameControl(new Button(11), "CheckLimitSwitch");
		listenerLeft.addButtonDownListener("CheckLimitSwitch", () -> {
            Log.info("Check Lift Limit Switch","" + liftLimitSwitch.get());
            Log.info("Check Four Bar Limit Switch","" + fourBarLimitSwitch.get());
        });
    }*/
    public static void main(String... args) {
        RobotBase.startRobot(MainWallace::new);
    }
}
   