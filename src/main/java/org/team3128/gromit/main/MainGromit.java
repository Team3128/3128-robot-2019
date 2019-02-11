package org.team3128.gromit.main;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.drive.SRXInvertCallback;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.hardware.misc.TwoSpeedGearshift;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.POVValue;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.listener.controltypes.POV;
import org.team3128.common.util.Constants;
import org.team3128.common.util.Log;

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

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.AnalogInput;


public class MainGromit extends NarwhalRobot{

    public AHRS ahrs;
    AnalogInput ai = new AnalogInput(0);
	public ADXRS450_Gyro gyro;
	
	// Drivetrain
	public SRXTankDrive drive;

	public TwoSpeedGearshift gearshift;
	public Piston gearshiftPiston;
	public double shiftUpSpeed, shiftDownSpeed;

    public double wheelCirc;
    public double gearRatio;
    public double wheelbase;
	public int driveMaxSpeed;
	
	public SRXInvertCallback teleopInvertCallback, autoInvertCallback;
	public double leftSpeedScalar, rightSpeedScalar;
    
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
	public DigitalInput softStopLimitSwitch;
	public int limitSwitchLocation, liftMaxVelocity;

	// Lift Intake
	public LiftIntake liftIntake;
	public LiftIntakeState liftIntakeState;
	public VictorSPX liftIntakeMotorLeader, liftIntakeMotorFollower;
	public Piston demogorgonPiston;

	// Optimus Prime!
	public OptimusPrime optimusPrime;

	// Climb
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
	public boolean override = false;

	public enum ManualControlMode {
        LIFT,
        FOUR_BAR;
    }
	ManualControlMode manualControMode = ManualControlMode.LIFT;
	
	public enum GameElement {
		CARGO,
		HATCH_PANEL;
	}
	GameElement currentGameElement = GameElement.CARGO;

	public enum ScoreLevel {
		TOP,
		MID,
		LOW;
	}
	ScoreLevel currentScoreLevel = ScoreLevel.LOW;

    @Override
    protected void constructHardware() {
		// Construct and Configure Drivetrain
		leftDriveLeader = new TalonSRX(10);
		leftDriveFollower = new VictorSPX(11);
		rightDriveLeader = new TalonSRX(15);
		rightDriveFollower = new VictorSPX(16);

		leftDriveLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
		leftDriveFollower.follow(leftDriveLeader);

        rightDriveLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
		rightDriveFollower.follow(rightDriveLeader);
		
		SRXTankDrive.initialize(rightDriveLeader, leftDriveLeader, wheelCirc, 1, wheelbase, driveMaxSpeed, teleopInvertCallback, autoInvertCallback);
        drive = SRXTankDrive.getInstance();

		drive.setLeftSpeedScalar(leftSpeedScalar);
		drive.setRightSpeedScalar(rightSpeedScalar);
		
		gearshift = new TwoSpeedGearshift(false, gearshiftPiston);
		drive.addShifter(gearshift, shiftUpSpeed, shiftDownSpeed);

		compressor = new Compressor();


		// Create Four-Bar
		fourBarState = FourBarState.LOW;
		fourBarMotor = new TalonSRX(30);

		FourBar.initialize(fourBarMotor, fourBarState);
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

		liftMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
		liftMotorFollower.follow(liftMotorLeader);

		Lift.initialize(liftState, liftMotorLeader, softStopLimitSwitch, limitSwitchLocation, liftMaxVelocity);
		lift = Lift.getInstance();


		// Create Lift Intake
		liftIntakeState = LiftIntake.LiftIntakeState.STOPPED;
		liftIntakeMotorLeader = new VictorSPX(31);
		liftIntakeMotorFollower = new VictorSPX(32);
		liftIntakeMotorFollower.set(ControlMode.Follower, liftIntakeMotorLeader.getDeviceID());
		liftIntakeMotorLeader.setInverted(true);

		LiftIntake.initialize(liftIntakeMotorLeader, liftIntakeState, demogorgonPiston, false);
		liftIntake = LiftIntake.getInstance();


		// Create Optimus Prime
		OptimusPrime.initialize();
		optimusPrime = OptimusPrime.getInstance();

		// Create the Climber
		climbMotor = new TalonSRX(40);
		climbMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);

		// Instantiate PDP
		powerDistPanel = new PowerDistributionPanel();

		ds = DriverStation.getInstance();

		// Instantiate gryoscopes
        ahrs = new AHRS(SPI.Port.kMXP); 
		ahrs.reset();
		
		gyro = new ADXRS450_Gyro();
		gyro.calibrate();

		// Setup listeners
        leftJoystick = new Joystick(1);
		listenerLeft = new ListenerManager(leftJoystick);
		addListenerManager(listenerLeft);

		rightJoystick = new Joystick(0);
		listenerRight = new ListenerManager(rightJoystick);
		addListenerManager(listenerRight);
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
			double x = listenerRight.getAxis("MoveForwards");
			double y = listenerRight.getAxis("MoveTurn");
			double t = listenerRight.getAxis("Throttle") * -1;

			drive.arcadeDrive(x, -0.8 * y, t, true);
		}, "MoveForwards", "MoveTurn", "Throttle");

		listenerRight.nameControl(new Button(2), "Gearshift");
		listenerRight.addButtonDownListener("Gearshift", drive::shift);

		// Optimus Prime Controls
		listenerRight.nameControl(new POV(0), "IntakePOV");
		listenerRight.addListener("IntakePOV", (POVValue pov) ->
		{
			switch (pov.getDirectionValue()) {
				case 3:
				case 4:
				case 5:
					(optimusPrime.new CmdEnterIntakeMode()).start();
					break;
				default:
					(optimusPrime.new CmdExitIntakeMode()).start();
					break;
			}
		});

		listenerRight.nameControl(ControllerExtreme3D.TRIGGER, "Score");
		listenerRight.addButtonDownListener("Score", () -> {
			optimusPrime.setState(RobotState.getOptimusState(currentGameElement, currentScoreLevel));
		});
		listenerRight.addButtonUpListener("Score", () -> {
			optimusPrime.setState(RobotState.REST);
		});

		// Game Element Controls
		listenerRight.nameControl(new Button(3), "SelectHatchPanel");
		listenerRight.addButtonDownListener("SelectHatchPanel", () -> {
			currentGameElement = GameElement.HATCH_PANEL;
		});

		listenerRight.nameControl(new Button(3), "SelectCargo");
		listenerRight.addButtonDownListener("SelectCargo", () -> {
			currentGameElement = GameElement.CARGO;
		});

		// Height Controls
		listenerRight.nameControl(new Button(7), "SelectTopLevel");
		listenerRight.addButtonDownListener("SelectTopLevel", () -> {
			currentScoreLevel = ScoreLevel.TOP;
		});

		listenerRight.nameControl(new Button(9), "SelectMidLevel");
		listenerRight.addButtonDownListener("SelectMidLevel", () -> {
			currentScoreLevel = ScoreLevel.MID;
		});

		listenerRight.nameControl(new Button(11), "SelectLowLevel");
		listenerRight.addButtonDownListener("SelectLowLevel", () -> {
			currentScoreLevel = ScoreLevel.LOW;
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
        listenerLeft.addButtonDownListener("Override", () -> {
            override = true;
        });
        listenerLeft.addButtonUpListener("Override", () -> {
            override = false;
		});
		
		listenerLeft.nameControl(new Button(2), "ManualMode");
        listenerLeft.addButtonDownListener("ManualMode", () -> {
            manualControMode = ManualControlMode.FOUR_BAR;
        });
        listenerLeft.addButtonUpListener("ManualMode", () -> {
            manualControMode = ManualControlMode.LIFT;
		});
		
		listenerLeft.nameControl(ControllerExtreme3D.JOYY, "ManualContol");
        listenerLeft.addListener("ManualContol", (double joy) -> {
            if (manualControMode == ManualControlMode.FOUR_BAR) {
                fourBar.override = true;
                fourBar.powerControl(joy * 0.5);
            }
            else {
                lift.override = override;
                lift.powerControl(joy);
            }
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
		
		listenerLeft.nameControl(new Button(7), "SetLiftZero");
		listenerLeft.addButtonDownListener("SetLiftZero", () ->
		{
			liftMotorLeader.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
		});


	}
	
	@Override
	protected void constructAutoPrograms()
	{

	}

	@Override
	protected void teleopInit()
	{

	}
	
	@Override
	protected void disabledInit()
	{

	}
	
	@Override
	protected void disabledPeriodic()
	{

	}

	@Override
	protected void autonomousInit()
	{
		
	}

	@Override
	protected void updateDashboard()
	{	
		
	}
}