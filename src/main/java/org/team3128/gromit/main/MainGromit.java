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
import org.team3128.gromit.mechanisms.GroundIntake;
import org.team3128.gromit.mechanisms.Lift;
import org.team3128.gromit.mechanisms.LiftIntake;
import org.team3128.gromit.mechanisms.OptimusPrime;
import org.team3128.gromit.mechanisms.FourBar.FourBarState;
import org.team3128.gromit.mechanisms.GroundIntake.GroundIntakeState;
import org.team3128.gromit.mechanisms.Lift.LiftHeight;
import org.team3128.gromit.mechanisms.LiftIntake.LiftIntakeState;

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
	public GroundIntake groundIntake;
	public GroundIntakeState groundIntakeState;
	public VictorSPX groundIntakeMotor;
	public Piston groundIntakePistons;

	// Lift
	public Lift lift;
	public LiftHeight liftState;
	public TalonSRX liftMotorLeader;
	public VictorSPX liftMotorFollower;
	public DigitalInput softStopLimitSwitch;
	public int limitSwitchLocation, liftMaxVelocity;

	// Lift Intake
	public LiftIntake liftIntake;
	public LiftIntakeState liftIntakeState;
	public VictorSPX liftIntakeMotorLeader, liftIntakeMotorFollower;
	public Piston liftPiston;

	// Optimus Prime!
	public OptimusPrime optimusPrime;

	// Controls
	public Joystick leftJoystick;
	public Joystick rightJoystick;
	public ListenerManager listenerLeft;
	public ListenerManager listenerRight;

	// Miscellaneous
	public PowerDistributionPanel powerDistPanel;

	public DriverStation ds;

    @Override
    protected void constructHardware() {
		// Construct and Configure Drivetrain
		leftDriveLeader = new TalonSRX(0);
		leftDriveFollower = new VictorSPX(1);
		rightDriveLeader = new TalonSRX(2);
		leftDriveFollower = new VictorSPX(3);

		leftDriveLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
		leftDriveFollower.set(ControlMode.Follower, leftDriveLeader.getDeviceID());

        rightDriveLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
		rightDriveFollower.set(ControlMode.Follower, rightDriveLeader.getDeviceID());
		
		SRXTankDrive.initialize(rightDriveLeader, leftDriveLeader, wheelCirc, 1, wheelbase, driveMaxSpeed, teleopInvertCallback, autoInvertCallback);
        drive = SRXTankDrive.getInstance();

		drive.setLeftSpeedScalar(leftSpeedScalar);
		drive.setRightSpeedScalar(rightSpeedScalar);
		
		shiftUpSpeed = 0;
		shiftDownSpeed = 0;
		gearshift = new TwoSpeedGearshift(false, gearshiftPiston);
		drive.addShifter(gearshift, shiftUpSpeed, shiftDownSpeed);

		compressor = new Compressor();


		// Create Four-Bar
		fourBarState = FourBarState.LOW;
		fourBarMotor = new TalonSRX(4);

		FourBar.initialize(fourBarMotor, fourBarState);
		fourBar = FourBar.getInstance();


		// Create Ground Intake
		groundIntakeState = GroundIntake.GroundIntakeState.RETRACTED;
		groundIntakeMotor = new VictorSPX(5);

		GroundIntake.initialize(groundIntakeMotor, groundIntakeState, groundIntakePistons, false);
		groundIntake = GroundIntake.getInstance();


		// Create Lift
		liftState = LiftHeight.BASE;
		liftMotorLeader = new TalonSRX(6);
		liftMotorFollower = new VictorSPX(7);

		Lift.initialize(liftState, liftMotorLeader, softStopLimitSwitch, limitSwitchLocation, liftMaxVelocity);
		lift = Lift.getInstance();


		// Create Lift Intake
		liftIntakeState = LiftIntake.LiftIntakeState.STOPPED;
		liftIntakeMotorLeader = new VictorSPX(8);
		liftIntakeMotorFollower = new VictorSPX(9);
		liftIntakeMotorFollower.set(ControlMode.Follower, liftIntakeMotorLeader.getDeviceID());
		liftIntakeMotorLeader.setInverted(true);

		LiftIntake.initialize(liftIntakeMotorLeader, liftIntakeState, liftPiston, false);
		liftIntake = LiftIntake.getInstance();


		// Create Optimus Prime
		OptimusPrime.initialize();
		optimusPrime = OptimusPrime.getInstance();


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
        listenerRight.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
		listenerRight.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
		listenerRight.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");
		listenerRight.addMultiListener(() ->
		{
			double x = listenerRight.getAxis("MoveForwards");
			double y = listenerRight.getAxis("MoveTurn");
			double t = listenerRight.getAxis("Throttle") * -1;

			drive.arcadeDrive(x, y, t, true);
		}, "MoveForwards", "MoveTurn", "Throttle");

		listenerRight.nameControl(new Button(2), "Gearshift");
		listenerRight.addButtonDownListener("Gearshift", drive::shift);

		listenerLeft.nameControl(ControllerExtreme3D.JOYY, "ManualLiftControl");
		listenerLeft.addListener("ManualLiftControl", (double joyY) ->
		{
			lift.powerControl(joyY);
		});

		listenerLeft.nameControl(new Button(6), "FourBarUp");
		listenerLeft.addButtonDownListener("FourBarUp", () -> {
			fourBar.powerControl(0.5);
		});

		listenerLeft.nameControl(new Button(4), "FourBarDown");
		listenerLeft.addButtonUpListener("FourBarDown", () -> {
			fourBar.powerControl(-0.5);
		});
		
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

		listenerLeft.nameControl(new Button(7), "ForceLiftZero");
		listenerLeft.addButtonDownListener("ForceLiftZero", () ->
		{
			liftMotorLeader.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
		});

		listenerRight.nameControl(new Button(11), "StartCompressor");
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