package org.team3128.gromit.main;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.drive.SRXInvertCallback;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.drive.calibrationutility.DriveCalibrationUtility;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.hardware.misc.TwoSpeedGearshift;
import org.team3128.common.hardware.navigation.AnalogDevicesGyro;
import org.team3128.common.hardware.navigation.Gyro;
import org.team3128.common.hardware.navigation.NavX;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.POVValue;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.listener.controltypes.POV;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.util.Constants;
import org.team3128.common.util.Log;
import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Angle;
import org.team3128.common.util.units.Length;
import org.team3128.gromit.cvcommands.CmdSimpleCVServo;
import org.team3128.gromit.mechanisms.Climber;
import org.team3128.gromit.mechanisms.FourBar;
// import org.team3128.gromit.mechanisms.GroundIntake;
import org.team3128.gromit.mechanisms.Lift;
import org.team3128.gromit.mechanisms.LiftIntake;
import org.team3128.gromit.mechanisms.OptimusPrime;
import org.team3128.gromit.mechanisms.FourBar.FourBarState;
// import org.team3128.gromit.mechanisms.GroundIntake.GroundIntakeState;
import org.team3128.gromit.mechanisms.Lift.LiftHeightState;
import org.team3128.gromit.mechanisms.LiftIntake.LiftIntakeState;
import org.team3128.gromit.mechanisms.OptimusPrime.RobotState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class MainGromit extends NarwhalRobot{

    public Gyro gyro;

	// Drivetrain
	public SRXTankDrive drive;

	public TwoSpeedGearshift gearshift;
	public Piston gearshiftPiston;
	public double shiftUpSpeed, shiftDownSpeed;

    public double wheelCirc;
    //public double gearRatio;
    public double wheelbase;
	public int driveMaxSpeed;

	public SRXInvertCallback driveInvertCallback;
	public double leftSpeedScalar, rightSpeedScalar;

	public DriveCalibrationUtility dcu;

    // Drive Motors
	public TalonSRX leftDriveLeader;
	public VictorSPX leftDriveFollower;
	public TalonSRX rightDriveLeader;
	public VictorSPX rightDriveFollower;

	// Pneumatics
	public Compressor compressor;

	// Four-Bar
	public FourBar fourBar;
	public FourBarState fourBarState;
	public TalonSRX fourBarMotor;
	public DigitalInput fourBarLimitSwitch;
	public double fourBarSwitchPosition;
	public int fourBarMaxVelocity;

	// Ground Intake
	// public GroundIntake groundIntake;
	// public GroundIntakeState groundIntakeState;
	public VictorSPX groundIntakeMotor;
	public Piston groundIntakePistons;

	// Lift
	public Lift lift;
	public LiftHeightState liftState;
	public TalonSRX liftMotorLeader;
	public VictorSPX liftMotorFollower;
	public DigitalInput liftLimitSwitch;
	public int liftSwitchPosition, liftMaxVelocity;

	// Lift Intake
	public LiftIntake liftIntake;
	public LiftIntakeState liftIntakeState;
	public VictorSPX liftIntakeMotor;
	public Piston demogorgonPiston;
	public DigitalInput cargoBumperSwitch;

	// Optimus Prime!
	public OptimusPrime optimusPrime;

	// Climb
	public Climber climber;
	public Piston climbPiston;
	public TalonSRX climbMotor;

	

	// Controls
	public Joystick leftJoystick;
	public Joystick rightJoystick;
	public ListenerManager listenerLeft;
	public ListenerManager listenerRight;

	// Miscellaneous
	public PowerDistributionPanel powerDistPanel;
	public DriverStation ds;

	private CommandGroup climbCommand;

	// CV!!!!!!
	public Limelight limelight;

	// 4200
	public double maxLiftSpeed = 0;

	// 7510
	public double minLiftSpeed = 0;

	public enum ManualControlMode {
        LIFT,
        FOUR_BAR;
    }
	ManualControlMode manualControMode = ManualControlMode.LIFT;

	public enum GameElement {
		//NONE("none"),
		CARGO("cargo"),
		HATCH_PANEL("hatch_panel");

		private String name;
		private GameElement(String name) {
			this.name = name;
		}

		public String getName() {
			return name;
		}
	}
	GameElement currentGameElement = GameElement.CARGO;

	public enum ScoreTarget {
		ROCKET_TOP("top"),
		ROCKET_MID("mid"),
		ROCKET_LOW("low"),
		CARGO_SHIP("ship");

		private String name;
		private ScoreTarget(String name) {
			this.name = name;
		}

		public String getName() {
			return name;
		}
	}
 	ScoreTarget currentScoreTarget = ScoreTarget.CARGO_SHIP;

    @Override
    protected void constructHardware() {
		// Construct and Configure Drivetrain
		leftDriveLeader = new TalonSRX(15);
		leftDriveFollower = new VictorSPX(16);
		rightDriveLeader = new TalonSRX(10);
		rightDriveFollower = new VictorSPX(11);

		leftDriveLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
		leftDriveFollower.follow(leftDriveLeader);

        rightDriveLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
		rightDriveFollower.follow(rightDriveLeader);

		SRXTankDrive.initialize(rightDriveLeader, leftDriveLeader, wheelCirc, wheelbase, driveMaxSpeed, driveInvertCallback);
        drive = SRXTankDrive.getInstance();

		drive.setLeftSpeedScalar(leftSpeedScalar);
		drive.setRightSpeedScalar(rightSpeedScalar);

		gearshift = new TwoSpeedGearshift(false, gearshiftPiston);
		drive.addShifter(gearshift, shiftUpSpeed, shiftDownSpeed);

		// Instantiate gryoscopes

		// Instatiator if we're using the NavX
		gyro = new NavX();

		// Instatiator if we're using the KoP Gyro
		// gyro = new AnalogDevicesGyro();
		// ((AnalogDevicesGyro) gyro).recalibrate();

		// DCU
		//DriveCalibrationUtility.initialize(gyro);
		dcu = DriveCalibrationUtility.getInstance();

		compressor = new Compressor();

		// Create the Climber
		climbMotor = new TalonSRX(40);
		Climber.initialize(climbPiston, climbMotor);
		climber = Climber.getInstance();

		// Create Four-Bar
		fourBarMotor = new TalonSRX(30);

		FourBar.initialize(fourBarMotor, fourBarState, fourBarLimitSwitch, fourBarSwitchPosition, fourBarMaxVelocity);
		fourBar = FourBar.getInstance();

		// Create Ground Intake
		// groundIntakeState = GroundIntake.GroundIntakeState.RETRACTED;
		// groundIntakeMotor = new VictorSPX(99);

		// GroundIntake.initialize(groundIntakeMotor, groundIntakeState, groundIntakePistons, false);
		// groundIntake = GroundIntake.getInstance();

		// Create Lift
		liftState = LiftHeightState.BASE;
		liftMotorLeader = new TalonSRX(20);
		liftMotorFollower = new VictorSPX(21);

		liftMotorFollower.follow(liftMotorLeader);

		Lift.initialize(liftState, liftMotorLeader, liftLimitSwitch, liftMaxVelocity);
		lift = Lift.getInstance();


		// Create Lift Intake
		liftIntakeState = LiftIntake.LiftIntakeState.DEMOGORGON_HOLDING;
		liftIntakeMotor = new VictorSPX(31);

		LiftIntake.initialize(liftIntakeMotor, liftIntakeState, demogorgonPiston, cargoBumperSwitch);
		liftIntake = LiftIntake.getInstance();

		// Create Optimus Prime
		OptimusPrime.initialize();
		optimusPrime = OptimusPrime.getInstance();

		// Instantiate PDP
		powerDistPanel = new PowerDistributionPanel();

		ds = DriverStation.getInstance();

		// Setup listeners
        leftJoystick = new Joystick(1);
		listenerLeft = new ListenerManager(leftJoystick);
		addListenerManager(listenerLeft);

		rightJoystick = new Joystick(0);
		listenerRight = new ListenerManager(rightJoystick);
		addListenerManager(listenerRight);

		limelight = new Limelight(7 * Length.in, 26 * Length.in, 6.15 * Length.in, 28.5 * Length.in, 14.5 * Length.in);

		// NarwhalDashboard: Driver Controls
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

		NarwhalDashboard.addButton("setTarget_rocket_top", (boolean down) -> {
			if (down) {
				currentScoreTarget = ScoreTarget.ROCKET_TOP;
			}
		});
		NarwhalDashboard.addButton("setTarget_rocket_mid", (boolean down) -> {
			if (down) {
				currentScoreTarget = ScoreTarget.ROCKET_MID;
			}
		});
		NarwhalDashboard.addButton("setTarget_rocket_low", (boolean down) -> {
			if (down) {
				currentScoreTarget = ScoreTarget.ROCKET_LOW;
			}
		});
		NarwhalDashboard.addButton("setTarget_cargo_ship", (boolean down) -> {
			if (down) {
				currentScoreTarget = ScoreTarget.CARGO_SHIP;
			}
		});

		NarwhalDashboard.addButton("setElement_cargo", (boolean down) -> {
			if (down) {
				currentGameElement = GameElement.CARGO;
			}
		});
		NarwhalDashboard.addButton("setElement_hatch", (boolean down) -> {
			if (down) {
				currentGameElement = GameElement.HATCH_PANEL;
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

		NarwhalDashboard.addButton("fourbar_high", (boolean down) -> {
			if (down) {
				fourBar.setState(FourBarState.CARGO_HIGH);
			}
		});
		NarwhalDashboard.addButton("fourbar_ship_low", (boolean down) -> {
			if (down) {
				fourBar.setState(FourBarState.SHIP_AND_LOADING);
			}
		});
		NarwhalDashboard.addButton("fourbar_rocket_low", (boolean down) -> {
			if (down) {
				fourBar.setState(FourBarState.ROCKET_LOW);
			}
		});
		NarwhalDashboard.addButton("fourbar_intake", (boolean down) -> {
			if (down) {
				fourBar.setState(FourBarState.CARGO_INTAKE);
			}
		});

		NarwhalDashboard.addButton("lift_base", (boolean down) -> {
			if (down) {
				lift.setState(LiftHeightState.BASE);
			}
		});
		NarwhalDashboard.addButton("lift_loadship", (boolean down) -> {
			if (down) {
				lift.setState(RobotState.getOptimusState(currentGameElement, ScoreTarget.CARGO_SHIP).targetLiftState);
			}
		});
		NarwhalDashboard.addButton("lift_low", (boolean down) -> {
			if (down) {
				lift.setState(RobotState.getOptimusState(currentGameElement, ScoreTarget.ROCKET_LOW).targetLiftState);
			}
		});
		NarwhalDashboard.addButton("lift_mid", (boolean down) -> {
			if (down) {
				lift.setState(RobotState.getOptimusState(currentGameElement, ScoreTarget.ROCKET_MID).targetLiftState);
			}
		});
		NarwhalDashboard.addButton("lift_top", (boolean down) -> {
			if (down) {
				lift.setState(RobotState.getOptimusState(currentGameElement, ScoreTarget.ROCKET_TOP).targetLiftState);
			}
		});

		NarwhalDashboard.addButton("arc_90_r_40", (boolean down) -> {
			if (down) {
				drive.new CmdArcTurn(40 * Length.in, 90 * Angle.DEGREES, Direction.RIGHT, 0.8, 5000).start();;
			}
		});
		NarwhalDashboard.addButton("arc_90_r_30", (boolean down) -> {
			if (down) {
				drive.new CmdArcTurn(30 * Length.in, 90 * Angle.DEGREES, Direction.RIGHT, 0.8, 5000).start();;
			}
		});
		NarwhalDashboard.addButton("simplecv", (boolean down) -> {
			if (down) {
				new CmdSimpleCVServo(limelight).start();
			}
		});

		//dcu.initNarwhalDashboard();
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
			double vert =     -1.0 * listenerRight.getAxis("MoveForwards");
			double horiz =    -0.8 * listenerRight.getAxis("MoveTurn");
			double throttle = -1.0 * listenerRight.getAxis("Throttle");

			drive.arcadeDrive(vert, horiz, throttle, true);
		}, "MoveForwards", "MoveTurn", "Throttle");

		listenerRight.nameControl(new Button(2), "Gearshift");
		listenerRight.addButtonDownListener("Gearshift", drive::shift);

		// Intake/Outtake Controls
		listenerRight.nameControl(new Button(5), "DemogorgonGrab");
        listenerRight.addButtonDownListener("DemogorgonGrab", () -> {
			liftIntake.setState(LiftIntakeState.DEMOGORGON_RELEASED);
		});
		listenerRight.addButtonUpListener("DemogorgonGrab", () -> {
			liftIntake.setState(LiftIntakeState.DEMOGORGON_HOLDING);
		});

		listenerRight.nameControl(new POV(0), "IntakePOV");
		listenerRight.addListener("IntakePOV", (POVValue pov) ->
		{
			switch (pov.getDirectionValue()) {
				case 8:
				case 1:
				case 7:
					liftIntake.setState(LiftIntakeState.CARGO_OUTTAKE);

					break;
				case 3:
				case 4:
				case 5:
					if (optimusPrime.robotState != RobotState.LOADING_AND_SHIP_HATCH) {
						optimusPrime.setState(RobotState.INTAKE_FLOOR_CARGO);
					}

					liftIntake.setState(LiftIntakeState.CARGO_INTAKE);
					
					break;
				case 0:
					liftIntake.setState(LiftIntakeState.CARGO_HOLDING);

					break;
				default:
					break;
			}
		});


		// Optimus Prime Controls
		listenerRight.nameControl(ControllerExtreme3D.TRIGGER, "SetHeight");
		listenerRight.addButtonDownListener("SetHeight", () -> {
			optimusPrime.setState(RobotState.getOptimusState(currentGameElement, currentScoreTarget));
		});
		listenerRight.addButtonUpListener("SetHeight", () -> {
			optimusPrime.setState(RobotState.REST);
		});

		listenerRight.nameControl(new Button(6), "RestState");
		listenerRight.addButtonDownListener("RestState" , () -> {
			optimusPrime.setState(RobotState.REST);
		});

		// Game Element Controls
		listenerRight.nameControl(new Button(4), "SelectHatchPanel");
		listenerRight.addButtonDownListener("SelectHatchPanel", () -> {
			currentGameElement = GameElement.HATCH_PANEL;
		});

		listenerRight.nameControl(new Button(3), "SelectCargo");
		listenerRight.addButtonDownListener("SelectCargo", () -> {
			currentGameElement = GameElement.CARGO;
		});

		// Scoring Structure Controls
		listenerRight.nameControl(new Button(8), "SelectCargoShip");
		listenerRight.addButtonDownListener("SelectCargoShip", () -> {
			currentScoreTarget = ScoreTarget.CARGO_SHIP;
		});

		// Height Controls
		listenerRight.nameControl(new Button(7), "SelectTopLevel");
		listenerRight.addButtonDownListener("SelectTopLevel", () -> {
			currentScoreTarget = ScoreTarget.ROCKET_TOP;
		});

		listenerRight.nameControl(new Button(9), "SelectMidLevel");
		listenerRight.addButtonDownListener("SelectMidLevel", () -> {
			currentScoreTarget = ScoreTarget.ROCKET_MID;
		});

		listenerRight.nameControl(new Button(11), "SelectLowLevel");
		listenerRight.addButtonDownListener("SelectLowLevel", () -> {
			currentScoreTarget = ScoreTarget.ROCKET_LOW;
		});

		// Compressor
		listenerRight.nameControl(new Button(10), "StartCompressor");
		listenerRight.addButtonDownListener("StartCompressor", () ->
		{
			compressor.start();
			Log.info("MainGuido", "Starting Compressor");
		});

		listenerRight.nameControl(new Button(12), "StopCompressor");
		listenerRight.addButtonDownListener("StopCompressor", () ->
		{
			compressor.stop();
			Log.info("MainGuido", "Stopping Compressor");
		});

		// MANUAL CONTROLS AND OVERRIDES

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


		listenerLeft.nameControl(new POV(0), "ManualIntakePOV");
        listenerLeft.addListener("ManualIntakePOV", (POVValue povVal) -> {
            switch (povVal.getDirectionValue()) {
                case 8:
                case 1:
                case 7:
					liftIntake.setState(LiftIntakeState.CARGO_OUTTAKE);
					break;
                case 3:
                case 4:
                case 5:
					liftIntake.setState(LiftIntakeState.CARGO_INTAKE);
					break;
                default:
					liftIntake.setState(LiftIntakeState.CARGO_HOLDING);
					break;
            }
		});

		listenerLeft.nameControl(new Button(7), "Climb1to2");
        listenerLeft.addButtonDownListener("Climb1to2", () -> {
            climber.new CmdClimb1to2().start();
        });

        listenerLeft.nameControl(new Button(8), "Climb2to3");
        listenerLeft.addButtonDownListener("Climb1to2", () -> {
            climber.new CmdClimb2to3().start();
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

	@Override
	protected void constructAutoPrograms()
	{
		NarwhalDashboard.addAuto("90 In Place", drive.new CmdInPlaceTurn(90, Direction.RIGHT, 1.0, 5000));
	}

	@Override
	protected void teleopInit()
	{
		drive.shiftToLow();

		fourBar.brake();
		lift.powerControl(0);
	}

	@Override
	protected void autonomousInit() {
		drive.shiftToLow();
	}


	private GameElement lastAutoGameElement = null;
	@Override
	protected void teleopPeriodic() {
		if (getCurrentGameElement() != lastAutoGameElement) {
			currentGameElement = getCurrentGameElement();
			lastAutoGameElement = currentGameElement;
		}
	}

	public GameElement getCurrentGameElement() {
		if (liftIntake.currentState == LiftIntakeState.CARGO_INTAKE || liftIntake.currentState == LiftIntakeState.CARGO_HOLDING) {
			return GameElement.CARGO;
		}
		else {
			return GameElement.HATCH_PANEL;
		}
	}


	@Override
	protected void updateDashboard()
	{
		//Log.info("roll, pitch", String.valueOf(gyro.getRoll()) + ", " + String.valueOf(gyro.getPitch()));
		SmartDashboard.putNumber("pitch", gyro.getPitch());
		SmartDashboard.putBoolean("Lift: Can Raise", lift.canRaise);
		SmartDashboard.putBoolean("Lift: Can Lower", lift.canLower);

		SmartDashboard.putNumber("Lift Height (inches)", lift.getCurrentHeight() / Length.in);
		SmartDashboard.putNumber("Lift Position", liftMotorLeader.getSelectedSensorPosition() / Length.in);


		SmartDashboard.putBoolean("Four Bar: Can Raise", fourBar.canRaise);
		SmartDashboard.putBoolean("Four Bar: Can Lower", fourBar.canLower);

		SmartDashboard.putNumber("Four Bar Angle (degrees)", fourBar.getCurrentAngle());

		SmartDashboard.putNumber("Back Leg Position (nu)", climbMotor.getSelectedSensorPosition());

		maxLiftSpeed = Math.max(maxLiftSpeed, liftMotorLeader.getSelectedSensorVelocity());
		SmartDashboard.putNumber("Max Upward Lift Speed", maxLiftSpeed);

		minLiftSpeed = Math.min(minLiftSpeed, liftMotorLeader.getSelectedSensorVelocity());
		SmartDashboard.putNumber("Min Lift Speed", minLiftSpeed);

		SmartDashboard.putNumber("Left Position (nu)", leftDriveLeader.getSelectedSensorPosition());
		SmartDashboard.putNumber("Right Position (nu)", rightDriveLeader.getSelectedSensorPosition());


		NarwhalDashboard.put("scoring_target", currentScoreTarget.getName());
		NarwhalDashboard.put("game_element", currentGameElement.getName());

		NarwhalDashboard.put("gear", drive.isInHighGear());

		dcu.tickNarwhalDashboard();
	}
}