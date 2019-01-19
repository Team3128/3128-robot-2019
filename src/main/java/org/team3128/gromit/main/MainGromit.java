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

public class MainGromit extends NarwhalRobot{
    public double wheelCirc;
    public double gearRatio;
    public double wheelBase;
    public double track;
    public int robotFreeSpeed;
    public SRXTankDrive drive;
    
    //Motors 
    public TalonSRX leftDriveFront;
    public VictorSPX leftDriveMiddle;
    public TalonSRX leftDriveBack;
    public TalonSRX rightDriveFront;
    public VictorSPX rightDriveMiddle;
    public TalonSRX rightDriveBack;


    public ListenerManager listenerRight;
    public ListenerManager listenerLeft;

	public Joystick leftJoystick;
    public Joystick rightJoystick;
    
    public Forklift forklift;
	public TalonSRX forkliftMotorLeader, forkliftMotorFollower;
    DigitalInput forkliftSoftStopLimitSwitch;
    
    Intake intake;
	IntakeState intakeState;
	Piston intakePiston;
	public VictorSPX intakeMotorLeader, intakeMotorFollower;
	DigitalInput intakeLimitSwitch;
	
	boolean intakeInverted;
    //Probably do some other stuff here idk
    /*
	
	public ADXRS450_Gyro gyro;

	public TwoSpeedGearshift gearshift;
	public Piston gearshiftPiston, climberPiston, climberLockPiston;
	
	public double shiftUpSpeed, shiftDownSpeed;

	public int lowGearMaxSpeed;

	// Pneumatics
	public Compressor compressor;

	public int limitSiwtchLocation, forkliftMaxVelocity;

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

    */
    @Override
    protected void constructHardware() {
        leftDriveFront = new TalonSRX(0);
        leftDriveMiddle = new VictorSPX(0);
        leftDriveBack = new TalonSRX(0);
        rightDriveFront = new TalonSRX(0);
        rightDriveMiddle = new VictorSPX(0);
        rightDriveBack = new TalonSRX(0);

		leftDriveFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
        rightDriveFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
        
        leftDriveMiddle.set(ControlMode.Follower, leftDriveFront.getDeviceID());
        leftDriveBack.set(ControlMode.Follower, leftDriveFront.getDeviceID());
        rightDriveMiddle.set(ControlMode.Follower, rightDriveFront.getDeviceID());
        rightDriveBack.set(ControlMode.Follower, rightDriveFront.getDeviceID());

        SRXTankDrive.initialize(leftDriveFront, rightDriveFront, wheelCirc, gearRatio, wheelBase, track, robotFreeSpeed);
        drive = SRXTankDrive.getInstance();

        public final PowerDistributionPanel powerDistPanel;

        public final long startTimeMillis = 0;
        
        public DriverStation ds;
        public RobotController rc;
    
        public double forkliftHeight = 0;
        public double linearSpeed = 0;
    
        public final double lowGearRatio = 8 + 1.0/3;
        public final double highGearRatio = 3 + 2.0/3;
    
        public double speedMult;

        powerDistPanel = new PowerDistributionPanel();
        /*
        //Intake
        intakeState = Intake.IntakeState.STOPPED;
		intakeMotorLeader = new VictorSPX(1);
		intakeMotorFollower = new VictorSPX(2);

		intakeMotorFollower.set(ControlMode.Follower, intakeMotorLeader.getDeviceID());
        intakeMotorLeader.setInverted(true);
        
        Intake.initialize(intakeMotorLeader, intakeState, intakePiston, intakeInverted);
        intake = Intake.getInstance();
        
        //Borklift

        forkliftMotorLeader = new TalonSRX(30);
		forkliftMotorFollower = new TalonSRX(31);

		forkliftMotorLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
		forkliftMotorFollower.set(ControlMode.Follower, forkliftMotorLeader.getDeviceID());

		Forklift.initialize(ForkliftState.GROUND, forkliftMotorLeader, forkliftSoftStopLimitSwitch,
				limitSiwtchLocation, forkliftMaxVelocity);
        forklift = Forklift.getInstance();
        */

        leftJoystick = new Joystick(0);
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

    }

}