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
import org.team3128.gromit.mechanisms.FourBar;
import org.team3128.gromit.mechanisms.GroundIntake;
import org.team3128.gromit.mechanisms.Lift;
import org.team3128.gromit.mechanisms.LiftIntake;
import org.team3128.gromit.mechanisms.OptimusPrime;
import org.team3128.gromit.mechanisms.FourBar.FourBarState;
import org.team3128.gromit.mechanisms.GroundIntake.GroundIntakeState;
import org.team3128.gromit.mechanisms.Lift.LiftState;
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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

import org.team3128.common.util.RobotMath;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class MainGromit extends NarwhalRobot{

    public AHRS ahrs;
    AnalogInput ai = new AnalogInput(0);
	public ADXRS450_Gyro gyro;
	
	//Drive Train
	public TwoSpeedGearshift gearshift;
	public Piston gearshiftPiston;

    public double wheelCirc;
    public double gearRatio;
    public double wheelBase;
    public double track;
    public int robotFreeSpeed;
	public SRXTankDrive drive;

	public double shiftUpSpeed, shiftDownSpeed;
    
    //Motors 
	public TalonSRX leftDriveLeader;
	public VictorSPX leftDriveFollower;
	public TalonSRX rightDriveLeader;
	public VictorSPX rightDriveFollower;

	// Pneumatics
	public Compressor compressor;

	//Four-Bar
	public FourBar fourBar;
	public FourBarState fourBarState;
	public TalonSRX fourBarMotor;

	//Ground Intake
	public GroundIntake groundIntake;
	public GroundIntakeState groundIntakeState;
	public VictorSPX groundIntakeMotor;
	public Piston leftGroundIntakePiston, rightGroundIntakePiston;

	//Lift
	public Lift lift;
	public LiftState liftState;
	public TalonSRX liftMotorLeader;
	public VictorSPX liftMotorFollower;
	public DigitalInput softStopLimitSwitch;
	public int limitSwitchLocation, liftMaxVelocity;

	//Lift Intake
	public LiftIntake liftIntake;
	public LiftIntakeState liftIntakeState;
	public VictorSPX liftIntakeMotorLeader, liftIntakeMotorFollower;
	public Piston liftPiston;

	//Optimus Prime!!
	public OptimusPrime optimusPrime;

	//Controls
	public Joystick leftJoystick;
	public Joystick rightJoystick;
	public ListenerManager listenerLeft;
	public ListenerManager listenerRight;

	//PID
    public double PID_kF;
    public double PID_kP;
    public double speedScalar;

	// Misc(general)
	public PowerDistributionPanel powerDistPanel;

	public long startTimeMillis = 0;
	
	public DriverStation ds;
	public RobotController rc;

    @Override
    protected void constructHardware() {
		leftDriveLeader = new TalonSRX(0);
		leftDriveFollower = new VictorSPX(1);
		rightDriveLeader = new TalonSRX(2);
		leftDriveFollower = new VictorSPX(3);


		leftDriveLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
        rightDriveLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);

        leftDriveFollower.set(ControlMode.Follower, leftDriveLeader.getDeviceID());
		rightDriveFollower.set(ControlMode.Follower, rightDriveLeader.getDeviceID());

		wheelCirc = 12.42*Length.in;
        wheelBase = 68.107;
        robotFreeSpeed = 4200;
        SRXTankDrive.initialize(rightDriveLeader, leftDriveLeader, wheelCirc, 1, wheelBase, robotFreeSpeed);
        drive = SRXTankDrive.getInstance();
        speedScalar = 1;
		drive.setLeftSpeedScalar(speedScalar);
		
		shiftUpSpeed = 5.0 * Length.ft * 60 / wheelCirc;
		shiftDownSpeed = 4.0 * Length.ft * 60 / wheelCirc;
		gearshift = new TwoSpeedGearshift(false, gearshiftPiston);
		drive.addShifter(gearshift, shiftUpSpeed, shiftDownSpeed);

		compressor = new Compressor();

		//create four-bar
		fourBarState = FourBarState.HATCH_PICKUP;
		fourBarMotor = new TalonSRX(4);
		fourBar = new FourBar(fourBarMotor, fourBarState);

		//create ground intake
		groundIntakeState = GroundIntake.GroundIntakeState.RETRACTED;
		groundIntakeMotor = new VictorSPX(5);
		groundIntake = new GroundIntake(groundIntakeMotor, groundIntakeState, leftGroundIntakePiston, rightGroundIntakePiston, false);

		//create lift
		liftState = LiftState.GROUND;
		liftMotorLeader = new TalonSRX(6);
		liftMotorFollower = new VictorSPX(7);
		lift = new Lift(liftState, liftMotorLeader, softStopLimitSwitch, limitSwitchLocation, liftMaxVelocity);

		//create lift intake
		liftIntakeState = LiftIntake.LiftIntakeState.STOPPED;
		liftIntakeMotorLeader = new VictorSPX(8);
		liftIntakeMotorFollower = new VictorSPX(9);
		liftIntakeMotorFollower.set(ControlMode.Follower, liftIntakeMotorLeader.getDeviceID());
		liftIntakeMotorLeader.setInverted(true);
		liftIntake = new LiftIntake(liftIntakeMotorLeader, liftIntakeState, liftPiston, false);

		//create Optimus Prime
		optimusPrime = new OptimusPrime(lift, fourBar, groundIntake);

		//instantiate PDP
		powerDistPanel = new PowerDistributionPanel();

		//gyro
        ahrs = new AHRS(SPI.Port.kMXP); 
        ahrs.reset();
        gyro = new ADXRS450_Gyro();

		//set listeners
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

		listenerRight.nameControl(new Button(2), "GearShift");
		listenerRight.addButtonDownListener("GearShift", drive::shift);

		listenerLeft.addListener("ForkliftTest", (double joyY) ->
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
		
		listenerRight.nameControl(new POV(0), "GroundIntakePOV");
		listenerRight.addListener("GroundIntakePOV", (POVValue pov) ->
		{
			int val = pov.getDirectionValue();

			switch (val)
			{
			case 7:
			case 8:
			case 1:
				groundIntake.setState(GroundIntakeState.DEPLOYED_INTAKE);
				break;
			case 3:
			case 4:
			case 5:
				groundIntake.setState(GroundIntakeState.DEPLOYED);
				break;
			default:
				groundIntake.setState(GroundIntakeState.RETRACTED);
			}
		});
	}
	
	protected void constructAutoPrograms(SendableChooser<CommandGroup> programChooser)
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