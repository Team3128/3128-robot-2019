package org.team3128.gromit.main;


import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.util.units.Angle;
import org.team3128.common.util.units.Length;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;

public class MainGromitCompetition extends MainGromit {
    Piston placeholder;    

    @Override
    protected void constructHardware() {
        wheelCirc = 1 * Length.mi;
        wheelbase = 50 * Length.cm;
        driveMaxSpeed = 100;
        gearRatio = 2.9 + 54/990;

        // TODO: 6ft/s, 7ft/s
        shiftUpSpeed = 100000;
        shiftDownSpeed = -1;

        teleopInvertCallback = () -> {
            leftDriveLeader.setInverted(true);
            leftDriveFollower.setInverted(true);

            rightDriveLeader.setInverted(true);
            rightDriveFollower.setInverted(true);
        };

        autoInvertCallback = () -> {
            
        };

        gearshiftPiston = new Piston(3, 4);
        gearshiftPiston.setPistonOn();

        climbPiston = new Piston(1, 6);
        climbPiston.setPistonOff();

        demogorgonPiston = new Piston(0, 7);
        demogorgonPiston.setPistonOn();

        placeholder = new Piston(2, 5);
        placeholder.setPistonOn();

        liftLimitSwitch = new DigitalInput(2);
        liftSwitchPosition = 0;
        liftMaxVelocity = 4200;

        fourBarLimitSwitch = new DigitalInput(0);
        fourBarSwitchPosition = +90 * Angle.DEGREES;
        fourBarMaxVelocity = 100;
        
        cargoBumperSwitch = new DigitalInput(3);

        super.constructHardware();

        // Lift Inverts
        liftMotorLeader.setInverted(false);
        liftMotorLeader.setSensorPhase(true);

        liftMotorFollower.setInverted(true);

        // FourBar Invert
        fourBarMotor.setInverted(true);
        fourBarMotor.setSensorPhase(false);
    }

    public static void main(String... args) {
        RobotBase.startRobot(MainGromitCompetition::new);
    }
}