package org.team3128.gromit.main;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.hardware.misc.TwoSpeedGearshift;
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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;


import org.team3128.common.util.RobotMath;

import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class MainGromit extends NarwhalRobot{

    AHRS ahrs;
    AnalogInput ai = new AnalogInput(0);
    public ADXRS450_Gyro gyro;


    public double wheelCirc;
    public double gearRatio;
    public double wheelBase;
    public double track;
    public int robotFreeSpeed;
    public SRXTankDrive drive;
    
    //Motors 
    public TalonSRX leftDriveFront;
    public TalonSRX leftDriveBack;
    public TalonSRX rightDriveFront;
    public TalonSRX rightDriveBack;

	public Joystick Joystick;
    public ListenerManager listener;

    /*
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
    */

    /*
	public TwoSpeedGearshift gearshift;
	public Piston gearshiftPiston, climberPiston, climberLockPiston;
	
	public double shiftUpSpeed, shiftDownSpeed;

	public int lowGearMaxSpeed;

	// Pneumatics
	public Compressor compressor;

	 public int limitSiwtchLocation, forkliftMaxVelocity;
    */

    /* Forklift and intake
    public Forklift forklift;
	public TalonSRX forkliftMotorLeader, forkliftMotorFollower;
    DigitalInput forkliftSoftStopLimitSwitch;
    
    Intake intake;
	IntakeState intakeState;
	Piston intakePiston;
	public VictorSPX intakeMotorLeader, intakeMotorFollower;
	DigitalInput intakeLimitSwitch;
	
    boolean intakeInverted;
    
    */

    @Override
    protected void constructHardware() {
        leftDriveFront = new TalonSRX(0);
        leftDriveBack = new TalonSRX(0);
        rightDriveFront = new TalonSRX(0);
        rightDriveBack = new TalonSRX(0);

		leftDriveFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
        rightDriveFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);

        leftDriveBack.set(ControlMode.Follower, leftDriveFront.getDeviceID());
        rightDriveBack.set(ControlMode.Follower, rightDriveFront.getDeviceID());

        SRXTankDrive.initialize(leftDriveFront, rightDriveFront, wheelCirc, gearRatio, wheelBase, track, robotFreeSpeed);
        drive = SRXTankDrive.getInstance();

        ahrs = new AHRS(SPI.Port.kMXP); 
        ahrs.reset();
        gyro = new ADXRS450_Gyro();


        /*		
		shiftUpSpeed = 5.0 * Length.ft * 60 / wheelCirc;
		shiftDownSpeed = 4.0 * Length.ft * 60 / wheelCirc;
		
		gearshift = new TwoSpeedGearshift(false, gearshiftPiston);
		drive.addShifter(gearshift, shiftUpSpeed, shiftDownSpeed);
        */

        /*
        //Intake
        intakeState = Intake.IntakeState.STOPPED;
		intakeMotorLeader = new VictorSPX(1);
		intakeMotorFollower = new VictorSPX(2);

		intakeMotorFollower.set(ControlMode.Follower, intakeMotorLeader.getDeviceID());
        intakeMotorLeader.setInverted(true);
        
        Intake.initialize(intakeMotorLeader, intakeState, intakePiston, intakeInverted);
        intake = Intake.getInstance();
        
        //Forklift

        forkliftMotorLeader = new TalonSRX(30);
		forkliftMotorFollower = new TalonSRX(31);

		forkliftMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
		forkliftMotorFollower.set(ControlMode.Follower, forkliftMotorLeader.getDeviceID());

		Forklift.initialize(ForkliftState.GROUND, forkliftMotorLeader, forkliftSoftStopLimitSwitch,
				limitSiwtchLocation, forkliftMaxVelocity);
        forklift = Forklift.getInstance();
        */

        Joystick = new Joystick(0);
		listener = new ListenerManager(Joystick);
        addListenerManager(listener);
        /*
        ds = DriverStation.getInstance();

        speedMult = wheelCirc / 409.6 / 100.0;
        */
        
        /* Dashboard stuff
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
        */

    }

    @Override
    protected void setupListeners() {
        listener.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
		listener.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
		listener.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");

		listener.addMultiListener(() ->
		{
			double x = listener.getAxis("MoveForwards");
			double y = listener.getAxis("MoveTurn");
			double t = listener.getAxis("Throttle") * -1;
			drive.arcadeDrive(x, y, t, true);
        }, "MoveForwards", "MoveTurn", "Throttle");
        
        listener.nameControl(new Button(11), "Forward");
        listener.addButtonDownListener("Forward", () -> 
        {
            leftDriveFront.set(ControlMode.PercentOutput, 100);  
            rightDriveFront.set(ControlMode.PercentOutput, 100);
        });
        listener.addButtonUpListener("Forward", () ->
        {
            leftDriveFront.set(ControlMode.PercentOutput, 0);
            rightDriveFront.set(ControlMode.PercentOutput, 0);
        });
        listener.nameControl(new Button(12), "Backward");
        listener.addButtonDownListener("Backward", () ->
        {
            leftDriveFront.set(ControlMode.PercentOutput, -100);
            rightDriveFront.set(ControlMode.PercentOutput, -100);
        });
        listener.addButtonUpListener("Backward", () ->
        {
            leftDriveFront.set(ControlMode.PercentOutput, 0);
            rightDriveFront.set(ControlMode.PercentOutput, 0);
        });

        /*

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

		listenerLeft.nameControl(new Button(11), "ClearStickyFaults");
		listenerLeft.addButtonDownListener("ClearStickyFaults", powerDistPanel::clearStickyFaults);

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
		
		//		listenerLeft.nameControl(new Button(9), "FullDrive");
		listenerLeft.addButtonDownListener("FullDrive", () -> {
			drive.arcadeDrive(-1.0, 0, 1.0, true);
		});
		listenerLeft.addButtonUpListener("FullDrive", () -> {
			drive.arcadeDrive(0, 0, 1.0, true);
		});
        */
    }

}