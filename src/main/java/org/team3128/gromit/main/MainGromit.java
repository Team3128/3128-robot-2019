package org.team3128.gromit.main;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.util.units.Angle;
import org.team3128.common.util.units.Length;
import org.team3128.gromit.mechanisms.Climber;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MainGromit extends MainDeepSpaceRobot {
    Piston placeholder;

    // Climber
	public Climber climber;
	public Piston climbPiston;
    public TalonSRX climbMotor;
    
    private CommandGroup climbCommand;

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
        liftSwitchPosition = 170;
        liftMaxVelocity = 4200;

        fourBarLimitSwitch = new DigitalInput(0);
        fourBarRatio = 4550 / (180 * Angle.DEGREES);
        fourBarSwitchPosition = +90 * Angle.DEGREES;
        fourBarMaxVelocity = 100;
        
        cargoBumperSwitch = new DigitalInput(1);
        
        super.constructHardware();

        // Create the Climber
		climbMotor = new TalonSRX(40);
		Climber.initialize(climbPiston, climbMotor);
        climber = Climber.getInstance();
        
        NarwhalDashboard.addButton("climb_12", (boolean down) -> {
			if (down) {
				if (climbCommand != null) climbCommand.cancel();

				climbCommand = climber.new CmdClimb1to2();
				climbCommand.start();
			}
		});
		NarwhalDashboard.addButton("climb_23", (boolean down) -> {
			if (down) {
				if (climbCommand != null) climbCommand.cancel();

				climbCommand = climber.new CmdClimb2to3();
				climber.new CmdClimb2to3().start();
			}
		});
		NarwhalDashboard.addButton("cancel_climb", (boolean down) -> {
			if (down) {
				if (climbCommand != null) climbCommand.cancel();
				climbCommand = null;
			}
        });
        
        // Debug
		NarwhalDashboard.addButton("rezero_backleg", (boolean down) -> {
			if (down) {
				climbMotor.set(ControlMode.PercentOutput, -0.8);
			}
			else {
				climbMotor.setSelectedSensorPosition(0);
				climbMotor.set(ControlMode.PercentOutput, 0);
			}
		});

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
        super.setupListeners();

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

    @Override
    protected void updateDashboard() {
        super.updateDashboard();

        SmartDashboard.putNumber("Back Leg Position (nu)", climbMotor.getSelectedSensorPosition());
    }

    public static void main(String... args) {
        RobotBase.startRobot(MainGromit::new);
    }
}