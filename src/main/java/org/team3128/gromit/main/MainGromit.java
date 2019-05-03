package org.team3128.gromit.main;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.team3128.common.hardware.limelight.StreamMode;
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
    @Override
	public String getTag() {
		return "MainGromit";
    }
    
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
        wheelCirc = 12.01 * Length.in;

        leftSpeedScalar = 1.00;
        rightSpeedScalar = 1.00;

        // TODO: 6ft/s, 7ft/s
        shiftUpSpeed = 100000;
        shiftDownSpeed = -1;

        gearshiftPiston = new Piston(3, 4);
        gearshiftPiston.setPistonOn();

        climbPiston = new Piston(1, 6);
        climbPiston.setPistonOff();

        demogorgonPiston = new Piston(0, 7);

        placeholder = new Piston(2, 5);
        placeholder.setPistonOn();

        liftLimitSwitch = new DigitalInput(2);
        liftSwitchPosition = 40;//170;
        liftMaxVelocity = 4200;

        fourBarLimitSwitch = new DigitalInput(0);
        fourBarRatio = 4600 / (180 * Angle.DEGREES); //4550
        fourBarSwitchPosition = +96 * Angle.DEGREES;
        
        cargoBumperSwitch = new DigitalInput(1);

        bottomLLHeight = 6.15 * Length.in;
        bottomLLAngle =  38.0 * Angle.DEGREES;

        topLLHeight =      43 * Length.in;
        topLLAngle =    -12.0 * Angle.DEGREES;

        // Construct and Configure Drivetrain
		leftDriveLeader = new TalonSRX(10);
		leftDriveFollower = new VictorSPX(11);
		rightDriveLeader = new TalonSRX(15);
		rightDriveFollower = new VictorSPX(16);
        
        super.constructHardware();

        topLimelight.setStreamMode(StreamMode.DRIVER_CAMERA);

        leftDriveLeader.setInverted(InvertType.None);
        leftDriveFollower.setInverted(InvertType.FollowMaster);
        leftDriveLeader.setSensorPhase(false);

        rightDriveLeader.setInverted(InvertType.InvertMotorOutput);
        rightDriveFollower.setInverted(InvertType.FollowMaster);
        rightDriveLeader.setSensorPhase(true);

        /**
         * Climber Construction
         */
		climbMotor = new TalonSRX(40);
        Climber.initialize(climbPiston, climbMotor);
        
        climber = Climber.getInstance();

        addMechanism(climber);
        
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