package org.team3128.guido.main;

import org.team3128.guido.mechanisms.Forklift;
import org.team3128.guido.mechanisms.Forklift.ForkliftState;
import org.team3128.guido.mechanisms.Intake;
import org.team3128.guido.mechanisms.Intake.IntakeState;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.hardware.misc.TwoSpeedGearshift;
import org.team3128.common.hardware.navigation.AnalogDevicesGyro;
import org.team3128.common.hardware.navigation.Gyro;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.POVValue;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.listener.controltypes.POV;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.util.Constants;
import org.team3128.common.util.Log;
import org.team3128.common.util.units.Length;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

public class MainGuido extends NarwhalRobot
{
	public double auto_delay = 0;
	
	// Drive Train
	public double wheelCirc;
	public SRXTankDrive drive;
	public TalonSRX leftDriveLeader, leftDriveFollower;
	public TalonSRX rightDriveLeader, rightDriveFollower;
	
	public Gyro gyro;

	public TwoSpeedGearshift gearshift;
	public Piston gearshiftPiston, climberPiston, climberLockPiston;
	
	public double shiftUpSpeed, shiftDownSpeed;

	public int lowGearMaxSpeed;

	// Pneumatics
	public Compressor compressor;

	// Forklift
	public Forklift forklift;
	public TalonSRX forkliftMotorLeader, forkliftMotorFollower;
	DigitalInput forkliftSoftStopLimitSwitch;

	public int limitSiwtchLocation, forkliftMaxVelocity;

	// Intake
	Intake intake;
	IntakeState intakeState;
	Piston intakePiston;
	public VictorSPX intakeMotorLeader, intakeMotorFollower;
	DigitalInput intakeLimitSwitch;
	
	boolean intakeInverted;

	// Controls
	public ListenerManager listenerRight;
	public ListenerManager listenerLeft;

	public Joystick leftJoystick;
	public Joystick rightJoystick;

	// Misc(general)
	public PowerDistributionPanel powerDistPanel;

	public long startTimeMillis = 0;
	
	public DriverStation ds;
	public RobotController rc;

	public double forkliftHeight = 0;
	public double linearSpeed = 0;

	public final double lowGearRatio = 8 + 1.0/3;
	public final double highGearRatio = 3 + 2.0/3;

	public double speedMult;

	@Override
	protected void constructHardware()
	{
		auto_delay = 0;
				
		limitSiwtchLocation = 0;

		wheelCirc = 12.6 * Length.in;
		lowGearMaxSpeed = 3800;

		gearshiftPiston = new Piston(3, 4);
		intakePiston = new Piston(1, 6);

		climberPiston = new Piston(2, 5);
		climberLockPiston = new Piston(0, 7);

		climberLockPiston.setPistonOff();
		climberPiston.setPistonOn();
		// intakePiston.invertPiston();

		intakeInverted = true;
		forkliftSoftStopLimitSwitch = new DigitalInput(6);

		// Drive Train Setup
		leftDriveLeader = new TalonSRX(20);
		leftDriveFollower = new TalonSRX(21);
		rightDriveLeader = new TalonSRX(10);
		rightDriveFollower = new TalonSRX(11);

		// set Leaders
		leftDriveLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
		rightDriveLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
				Constants.CAN_TIMEOUT);

		// set Followers
		leftDriveFollower.set(ControlMode.Follower, leftDriveLeader.getDeviceID());
		rightDriveFollower.set(ControlMode.Follower, rightDriveLeader.getDeviceID());

		gyro = new AnalogDevicesGyro();
		
		// create SRXTankDrive
		SRXTankDrive.initialize(leftDriveLeader, rightDriveLeader, wheelCirc, 25.25 * Length.in, lowGearMaxSpeed);

		invertSetup();
		drive = SRXTankDrive.getInstance();
		
		shiftUpSpeed = 5.0 * Length.ft * 60 / wheelCirc;
		shiftDownSpeed = 4.0 * Length.ft * 60 / wheelCirc;
		
		gearshift = new TwoSpeedGearshift(false, gearshiftPiston);
		drive.addShifter(gearshift, shiftUpSpeed, shiftDownSpeed);

		// create intake
		intakeState = Intake.IntakeState.STOPPED;
		intakeMotorLeader = new VictorSPX(1);
		intakeMotorFollower = new VictorSPX(2);

		intakeMotorFollower.set(ControlMode.Follower, intakeMotorLeader.getDeviceID());
		intakeMotorLeader.setInverted(true);

		compressor = new Compressor();

		Intake.initialize(intakeMotorLeader, intakeState, intakePiston, intakeInverted);
		intake = Intake.getInstance();

		// create forklift
		forkliftMotorLeader = new TalonSRX(30);
		forkliftMotorFollower = new TalonSRX(31);

		forkliftMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
				Constants.CAN_TIMEOUT);
		forkliftMotorFollower.set(ControlMode.Follower, forkliftMotorLeader.getDeviceID());

		Forklift.initialize(ForkliftState.GROUND, forkliftMotorLeader, forkliftSoftStopLimitSwitch,
				limitSiwtchLocation, forkliftMaxVelocity);
		forklift = Forklift.getInstance();

		// instantiate PDP
		powerDistPanel = new PowerDistributionPanel();

		// set Listeners
		leftJoystick = new Joystick(1);
		listenerLeft = new ListenerManager(leftJoystick);
		addListenerManager(listenerLeft);

		rightJoystick = new Joystick(0);
		listenerRight = new ListenerManager(rightJoystick);
		addListenerManager(listenerRight);

		ds = DriverStation.getInstance();

		speedMult = wheelCirc / 409.6 / 100.0;
		
		NarwhalDashboard.addButton("rezero", (boolean down) -> {
			if (down) {
				forklift.override = true;
				forklift.powerControl(-0.5);
			}
			else {
				forkliftMotorLeader.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
				forklift.powerControl(0);
				forklift.override = false;
			}
		});

		NarwhalDashboard.addButton("start_compress", (boolean down) -> {
			if (down) {
				compressor.start();
			}
		});

		NarwhalDashboard.addButton("stop_compress", (boolean down) -> {
			if (down) {
				compressor.stop();
			}
		});


		forklift.maxHeight = 19000;

		// CameraServer cameraServer = CameraServer.getInstance();
		// UsbCamera camera = cameraServer.startAutomaticCapture(0);
		// camera.setFPS(10);
		// camera.setResolution(160, 120);
	}

	@Override
	protected void setupListeners()
	{
		listenerRight.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
		listenerRight.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
		listenerRight.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");

		listenerRight.addMultiListener(() ->
		{
			double x = listenerRight.getAxis("MoveTurn");
			double y = listenerRight.getAxis("MoveForwards");
			double t = listenerRight.getAxis("Throttle") * -1;
			drive.arcadeDrive(-1 * x, -1 * y, t, true);
		}, "MoveForwards", "MoveTurn", "Throttle");

		listenerRight.nameControl(new Button(2), "GearShift");
		listenerRight.addButtonDownListener("GearShift", drive::shift);

		listenerRight.nameControl(new POV(0), "IntakePOV");
		listenerRight.addListener("IntakePOV", (POVValue pov) ->
		{
			int val = pov.getDirectionValue();

			switch (val)
			{
			case 7:
			case 8:
			case 1:
				intake.setState(IntakeState.OUTTAKE);
				break;
			case 3:
			case 4:
			case 5:
				intake.setState(IntakeState.INTAKE);
				break;
			default:
				intake.setState(IntakeState.STOPPED);
			}
		});

		listenerRight.nameControl(ControllerExtreme3D.TRIGGER, "SoftDrop");
		listenerRight.addButtonDownListener("SoftDrop", () -> {
			intake.setState(IntakeState.SOFT_DROP);
		});
		listenerRight.addButtonUpListener("SoftDrop", () -> {
			intake.setState(IntakeState.STOPPED);
		});
		
		listenerRight.nameControl(new Button(5), "ForkliftRightUp");
		listenerRight.addButtonDownListener("ForkliftRightUp", () ->
		{
			forklift.powerControl(1.0);
		});
		listenerRight.addButtonUpListener("ForkliftRightUp", () ->
		{
			forklift.powerControl(0);
		});

		listenerRight.nameControl(new Button(3), "ForkliftRightDown");
		listenerRight.addButtonDownListener("ForkliftRightDown", () ->
		{
			forklift.powerControl(-0.7);
		});
		listenerRight.addButtonUpListener("ForkliftRightDown", () ->
		{
			forklift.powerControl(0.0);
		});
		
		listenerLeft.nameControl(new Button(7), "ZeroForklift");
		listenerLeft.addButtonDownListener("ZeroForklift", () ->
		{
			forkliftMotorLeader.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
		});
		
		listenerLeft.nameControl(ControllerExtreme3D.TRIGGER, "Override");
		listenerLeft.addButtonDownListener("Override", () -> {
			forklift.override = true;
		});
		listenerLeft.addButtonUpListener("Override", () -> {
			forklift.override = false;
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
		});

		listenerLeft.nameControl(ControllerExtreme3D.JOYY, "ForkliftTest");
		listenerLeft.addListener("ForkliftTest", (double joyY) ->
		{
			forklift.powerControl(joyY);
		});
		
		listenerLeft.nameControl(new Button(11), "ReZero");
		listenerLeft.addButtonDownListener("ReZero", () -> {
			forklift.override = true;
			forklift.powerControl(-0.5);
		});
		listenerLeft.addButtonUpListener("ReZero", () -> {
			forklift.override = false;
			forklift.powerControl(0);
		});
		
		listenerLeft.nameControl(new POV(0), "IntakePOV");
		listenerLeft.addListener("IntakePOV", (POVValue pov) ->
		{
			int val = pov.getDirectionValue();

			switch (val)
			{
			case 7:
			case 8:
			case 1:
				intake.setState(IntakeState.OUTTAKE);
				break;
			case 3:
			case 4:
			case 5:
				intake.setState(IntakeState.INTAKE);
				break;
			default:
				intake.setState(IntakeState.STOPPED);
				break;
			}
		});

		listenerLeft.nameControl(new Button(8), "BrakeClimber");
		listenerLeft.addButtonDownListener("BrakeClimber", () ->
		{
			climberLockPiston.invertPiston();
		});
		listenerRight.nameControl(new Button(7), "DeployBar");
		listenerRight.addButtonDownListener("DeployBar", () ->
		{
				climberPiston.invertPiston();
		});
	}

	@Override
	protected void constructAutoPrograms()
	{

	}

	public void invertSetup()
	{
		leftDriveLeader.setInverted(false);
		leftDriveFollower.setInverted(false);

		rightDriveLeader.setInverted(false);
		rightDriveFollower.setInverted(false);

		rightDriveFollower.setInverted(true);
		rightDriveLeader.setInverted(true);

		rightDriveLeader.setSensorPhase(true);
		leftDriveLeader.setSensorPhase(true);
	}

	@Override
	protected void teleopInit()
	{
		forklift.disabled = false;

		intakeState = IntakeState.STOPPED;

		leftDriveLeader.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
		rightDriveLeader.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);

		gearshift.shiftToHigh();
		if (gearshift.isInHighGear())
		{

			Log.info("MainGuido", "Log: Gearshift is in High Gear");

		}
		else
		{
			Log.info("MainGuido", "Log: Gearshift is in Low Gear");

		}

		startTimeMillis = System.currentTimeMillis();
	}

	@Override
	protected void teleopPeriodic()
	{

	}

	@Override
	protected void disabledInit()
	{
		forklift.disabled = true;
	}
	
	@Override
	protected void disabledPeriodic()
	{
	}

	@Override
	protected void autonomousInit()
	{
		forklift.disabled = false;
		drive.shiftToLow();
	}

	@Override
	protected void updateDashboard()
	{	
        NarwhalDashboard.put("time", DriverStation.getInstance().getMatchTime());
		NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());
		
		NarwhalDashboard.put("speed",
			speedMult * Math.abs(0.5*(rightDriveLeader.getSelectedSensorVelocity(0) + leftDriveLeader.getSelectedSensorVelocity(0)))
		);
		
		forkliftHeight = 10.25/12.0 + forkliftMotorLeader.getSelectedSensorPosition(0) / 262.95 / 12;
		NarwhalDashboard.put("height", forkliftHeight);
	}

	public static void main(String[] args) {
		RobotBase.startRobot(MainGuido::new);
	}
}