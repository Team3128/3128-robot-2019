package org.team3128.gromit.main;


import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.util.units.Length;

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

        super.constructHardware();

        // Lift Inverts
        liftMotorLeader.setInverted(false);
        liftMotorLeader.setSensorPhase(true);

        liftMotorLeader.setSelectedSensorPosition(0);

        liftMotorFollower.setInverted(true);

        // FourBar Invert
        fourBarMotor.setInverted(true);
        fourBarMotor.setSensorPhase(false);
    }

    public static void main(String... args) {
        RobotBase.startRobot(MainGromitCompetition::new);
    }
}