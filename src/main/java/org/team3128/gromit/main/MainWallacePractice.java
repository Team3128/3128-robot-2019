package org.team3128.gromit.main;


import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.util.units.Angle;
import org.team3128.common.util.units.Length;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.util.Log;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;

public class MainWallacePractice extends MainGromit {
    Piston placeholder;    
    

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

        driveInvertCallback = () -> {
            leftDriveLeader.setInverted(true);
            leftDriveFollower.setInverted(true);
            leftDriveLeader.setSensorPhase(true);

            rightDriveLeader.setInverted(true);
            rightDriveFollower.setInverted(true);
            rightDriveLeader.setSensorPhase(false);
        };

        gearshiftPiston = new Piston(3, 4);
        gearshiftPiston.setPistonOn();

        climbPiston = new Piston(1, 6);
        climbPiston.setPistonOff();

        demogorgonPiston = new Piston(7, 0);

        placeholder = new Piston(2, 5);
        placeholder.setPistonOn();

        liftLimitSwitch = new DigitalInput(2);
        liftSwitchPosition = 0;
        liftMaxVelocity = 4200;

        fourBarLimitSwitch = new DigitalInput(0);
        fourBarSwitchPosition = +90 * Angle.DEGREES;
        fourBarMaxVelocity = 100;
        
        cargoBumperSwitch = new DigitalInput(1);
        
        super.constructHardware();

        // Lift Inverts
        liftMotorLeader.setInverted(false);
        liftMotorLeader.setSensorPhase(true);

        liftMotorFollower.setInverted(true);

        // Lift Intake Invert
        liftIntakeMotor.setInverted(false);

        // FourBar Invert
        fourBarMotor.setInverted(true);
        fourBarMotor.setSensorPhase(false);

        // Climber Invert
        climbMotor.setSensorPhase(true);

        //2 is big camera for lars KEEP AT 2
        limelight.driverMode(2);
        limelight.turnOffLED();
    }

    @Override
    protected void setupListeners() {
		// REGULAR CONTROLS

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
	
                drive.arcadeDrive(-vert, horiz, throttle, true);
                Log.info("Joystick", "IT WORKS");
			}
        }, "MoveForwards", "MoveTurn", "Throttle");
    }
    public static void main(String... args) {
        RobotBase.startRobot(MainWallacePractice::new);
    }
}
   