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
        wheelbase = 37 * Length.in;
        driveMaxSpeed = 5800;
        //gearRatio = 2.9 + 54/990;
        wheelCirc = 6.55 * Length.in;

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
            rightDriveLeader.setSensorPhase(true);
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
        
        cargoBumperSwitch = new DigitalInput(3);

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
    }

    public static void main(String... args) {
        RobotBase.startRobot(MainGromitCompetition::new);
    }
}