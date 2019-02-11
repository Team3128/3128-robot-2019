package org.team3128.gromit.main;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.listener.POVValue;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.listener.controltypes.POV;
import org.team3128.common.util.units.Length;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;

public class MainGromitCompetition extends MainGromit {
    Piston notActuallyPiston;
    Solenoid[] sols = new Solenoid[8];

    boolean override = false;

    public enum ManualControlMode {
        Lift,
        FourBar;
    }
    ManualControlMode manualControMode = ManualControlMode.Lift;

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

        notActuallyPiston = new Piston(2, 5);
        notActuallyPiston.setPistonOn();

        super.constructHardware();

        liftMotorLeader.setInverted(true);
        liftMotorLeader.setSensorPhase(false);

        liftMotorLeader.setSelectedSensorPosition(0);
    }

    @Override
    protected void setupListeners() {
        super.setupListeners();

        listenerLeft.nameControl(ControllerExtreme3D.TRIGGER, "Override");
        listenerLeft.addButtonDownListener("Override", () -> {
            override = true;
        });
        listenerLeft.addButtonUpListener("Override", () -> {
            override = false;
        });

        listenerLeft.nameControl(ControllerExtreme3D.JOYY, "ManualContol");
        listenerLeft.addListener("ManualContol", (double joy) -> {
            if (manualControMode == ManualControlMode.FourBar) {
                fourBar.override = true;
                fourBar.powerControl(joy * 0.5);
            }
            else {
                lift.override = override;
                lift.powerControl(joy);
            }
        });

        listenerLeft.nameControl(new Button(2), "ManualMode");
        listenerLeft.addButtonDownListener("ManualMode", () -> {
            manualControMode = ManualControlMode.FourBar;
        });
        listenerLeft.addButtonUpListener("ManualMode", () -> {
            manualControMode = ManualControlMode.Lift;
        });

        listenerLeft.nameControl(new POV(0), "ManualIntakePOV");
        listenerLeft.addListener("ManualIntakePOV", (POVValue povVal) -> {
            switch (povVal.getDirectionValue()) {
                case 8:
                case 1:
                case 2:
                    liftIntakeMotorLeader.set(ControlMode.PercentOutput, +0.5);
                    break;
                case 4:
                case 5:
                case 6:
                    liftIntakeMotorLeader.set(ControlMode.PercentOutput, -0.5);
                    break;
                default:
                    liftIntakeMotorLeader.set(ControlMode.PercentOutput, 0);
                    break;
            }
        });

        listenerLeft.nameControl(new Button(5), "DemogorgonGrab");
        listenerLeft.addButtonDownListener("DemogorgonGrab", () -> {
            demogorgonPiston.setPistonOff();
        });
        listenerLeft.addButtonUpListener("DemogorgonGrab", () -> {
            demogorgonPiston.setPistonOn();
        });

        listenerLeft.nameControl(new Button(9), "ClimbPistonExtend");
        listenerLeft.addButtonDownListener("ClimbPistonExtend", () -> {
            climbPiston.setPistonOn();
        });

        listenerLeft.nameControl(new Button(10), "ClimbPistonRetract");
        listenerLeft.addButtonDownListener("ClimbPistonRetract", () -> {
            climbPiston.setPistonOff();
        });

        listenerLeft.nameControl(new Button(11), "BackLegDown");
        listenerLeft.nameControl(new Button(12), "BackLegUp");
        listenerLeft.addMultiListener(() -> {
            if (listenerLeft.getButton("BackLegDown") && 
               !listenerLeft.getButton("BackLegUp")) {
                climbMotor.set(ControlMode.PercentOutput, +1.0);
            }
            else if (listenerLeft.getButton("BackLegUp") &&
                    !listenerLeft.getButton("BackLegDown")) {
                climbMotor.set(ControlMode.PercentOutput, -1.0);
            }
            else {
                climbMotor.set(ControlMode.PercentOutput, 0.0);
            }
        }, "BackLegDown", "BackLegUp");
    }

    public static void main(String... args) {
        RobotBase.startRobot(MainGromitCompetition::new);
    }
}