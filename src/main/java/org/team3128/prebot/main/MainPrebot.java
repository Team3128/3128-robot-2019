package org.team3128.prebot.main;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team3128.prebot.autonomous.*;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.drive.SRXTankDrive.FeedForwardPowerMultiplier;
import org.team3128.common.drive.SRXTankDrive.FeedForwardPowerMultiplierSet;
import org.team3128.common.drive.SRXTankDrive.Wheelbase;
import org.team3128.common.drive.calibrationutility.DriveCalibrationUtility;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.navigation.Gyro;
import org.team3128.common.hardware.navigation.NavX;
import org.team3128.common.util.Constants;
import org.team3128.common.util.units.Length;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.datatypes.PIDConstants;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;

import org.team3128.common.hardware.limelight.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class MainPrebot extends NarwhalRobot {
    public TalonSRX rightDriveFront, rightDriveMiddle, rightDriveBack;
    public TalonSRX leftDriveFront, leftDriveMiddle, leftDriveBack;

    public SRXTankDrive tankDrive;

    public Joystick joystick;
    public ListenerManager lm;

    public Gyro gyro;

    public PIDConstants leftMotionProfilePID, leftVelocityPID;
    public PIDConstants rightMotionProfilePID, rightVelocityPID;

    public double speedScalar;

    public double maxLeftSpeed = 0;
    public double maxRightSpeed = 0;
    public double leftSpeed = 0;
    public double rightSpeed = 0;
    public NetworkTable table;
    public NetworkTable table2;

    public NetworkTable limelightTable;

    public DriveCalibrationUtility dcu;
    public Wheelbase calculatedWheelbase;

    public Limelight limelight = new Limelight(0 * Length.in, 26 * Length.in, 6.15 * Length.in, 28.5 * Length.in, 14.5 * Length.in);
	@Override
	protected void constructHardware()
	{
		limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

        rightDriveFront = new TalonSRX(0);
        rightDriveMiddle = new TalonSRX(1);
        rightDriveBack = new TalonSRX(2);

        leftDriveFront = new TalonSRX(3);
        leftDriveMiddle = new TalonSRX(4);
        leftDriveBack = new TalonSRX(5);

        rightDriveFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
        rightDriveMiddle.set(ControlMode.Follower, rightDriveFront.getDeviceID());
        rightDriveBack.set(ControlMode.Follower, rightDriveFront.getDeviceID());

        leftDriveFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
        leftDriveMiddle.set(ControlMode.Follower, leftDriveFront.getDeviceID());
        leftDriveBack.set(ControlMode.Follower, leftDriveFront.getDeviceID());

        double wheelCirc = 13.21 * Length.in;
        double wheelBase = 30 * Length.in;//68.61 * Length.in;
        int robotFreeSpeed = 3700;

        SRXTankDrive.initialize(rightDriveFront, leftDriveFront, wheelCirc, wheelBase, robotFreeSpeed,
            () -> {
                Log.info("SRXTankDrive", "Inverting drive motors.");

                leftDriveFront.setInverted(true);
                leftDriveMiddle.setInverted(true);
                leftDriveBack.setInverted(true);
                
                rightDriveFront.setInverted(false);
                rightDriveMiddle.setInverted(false);
                rightDriveBack.setInverted(false);

                leftDriveFront.setSensorPhase(true);
                rightDriveFront.setSensorPhase(true);
            });

        tankDrive = SRXTankDrive.getInstance();
        tankDrive.setLeftSpeedScalar(1.0);
        tankDrive.setRightSpeedScalar(0.9178);
        
        // Instatiator if we're using the NavX
		gyro = new NavX();

		// Instatiator if we're using the KoP Gyro
		// gyro = new AnalogDevicesGyro();
		// ((AnalogDevicesGyro) gyro).recalibrate();

        joystick = new Joystick(1);
		lm = new ListenerManager(joystick);
        addListenerManager(lm);

        // DCU
		DriveCalibrationUtility.initialize(gyro);
		dcu = DriveCalibrationUtility.getInstance();

        dcu.initNarwhalDashboard();

        int timeMs = 4000;
            /*
        NarwhalDashboard.addButton("g_10_08", (boolean down) -> {
            if (down) {
                Log.info("cmdfeedfrwrd", "triggered");
                tankDrive.new CmdGetFeedForwardPowerMultiplier(wrapper,ffpSetAvg,gyro,1.0,0.8,timeMs).start();
            }
        });
        NarwhalDashboard.addButton("g_10_06", (boolean down) -> {
            if (down) {
                tankDrive.new CmdGetFeedForwardPowerMultiplier(wrapper, gyro,1.0,0.6,timeMs).start();
            }
        });
        NarwhalDashboard.addButton("g_10_04", (boolean down) -> {
            if (down) {
                tankDrive.new CmdGetFeedForwardPowerMultiplier(wrapper,gyro,1.0,0.4,timeMs).start();
            }
        });
        NarwhalDashboard.addButton("g_10_02", (boolean down) -> {
            if (down) {
                tankDrive.new CmdGetFeedForwardPowerMultiplier(wrapper,gyro,1.0,0.2,timeMs).start();
            }
        });
        NarwhalDashboard.addButton("g_08_10", (boolean down) -> {
            if (down) {
                tankDrive.new CmdGetFeedForwardPowerMultiplier(wrapper,gyro,0.8,1.0,timeMs).start();
            }
        });
        NarwhalDashboard.addButton("g_06_10", (boolean down) -> {
            if (down) {
                tankDrive.new CmdGetFeedForwardPowerMultiplier(wrapper,gyro,0.6,1.0,timeMs).start();
            }
        });
        NarwhalDashboard.addButton("g_04_10", (boolean down) -> {
            if (down) {
                tankDrive.new CmdGetFeedForwardPowerMultiplier(wrapper,gyro,0.4,1.0,timeMs).start();
            }
        });
        NarwhalDashboard.addButton("g_02_10", (boolean down) -> {
            if (down) {
                tankDrive.new CmdGetFeedForwardPowerMultiplier(wrapper,gyro,0.2,1.0,timeMs).start();
            }
        });

        NarwhalDashboard.addButton("flushCSV", (boolean down) -> {
            if (down) {
                System.out.println(wrapper.getCSV());
            }
        });
        */
        
    }
    
    @Override
    protected void constructAutoPrograms() {
        NarwhalDashboard.addAuto("Turn", new CmdInPlaceTurnTest());
        NarwhalDashboard.addAuto("Arc Turn", new CmdArcTurnTest());
        NarwhalDashboard.addAuto("Forward", new CmdDriveForward());
        //NarwhalDashboard.addAuto("Test", new Test(tankDrive, ahrs));
        // NarwhalDashboard.addAuto("Wheel Base Test", new CmdCalibrateWheelbase(ahrs, 10, 1000, 1500, calculatedWheelbase));
        // NarwhalDashboard.addAuto("Forward CV", new CmdDriveForwardCVTest());
        // NarwhalDashboard.addAuto("Routemaker Test", new CmdRoutemakerTest());
        // NarwhalDashboard.addAuto("Heading Then Arc Turn", new CmdHeadingThenArc(limelight));
        NarwhalDashboard.addAuto("Fancy Wheel Base Calibration", new CmdFancyCalibrateWheelBase(gyro));
        // previous speeds that were used were 2000, 4000 (arbitrarily picked)
    }

	@Override
	protected void setupListeners() {
        lm.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
		lm.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
		lm.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");		

        lm.addMultiListener(() -> {
			tankDrive.arcadeDrive(
                -0.7 * RobotMath.thresh(lm.getAxis("MoveTurn"), 0.1),
                -1.0 * RobotMath.thresh(lm.getAxis("MoveForwards"), 0.1),
                -1.0 * lm.getAxis("Throttle"),
                 true
            );		
        }, "MoveTurn", "MoveForwards", "Throttle");

        lm.nameControl(new Button(12), "FullSpeed");
        lm.addButtonDownListener("FullSpeed", () ->
		{
			tankDrive.tankDrive(-1, -1);
        });
        lm.addButtonUpListener("FullSpeed", () ->
		{
			tankDrive.tankDrive(0, 0);
		});


        lm.nameControl(new Button(2), "LightOn");
		lm.addButtonDownListener("LightOn", () -> {
            limelightTable.getEntry("ledMode").setNumber(3);
            Log.debug("Limelight Latency", String.valueOf(limelightTable.getEntry("tl").getDouble(0.0)));
  
        });

		lm.nameControl(ControllerExtreme3D.TRIGGER, "LogLimelight");
		lm.addButtonDownListener("LogLimelight", () -> { 
        });
    }

    @Override
    protected void teleopPeriodic() {
    }

    @Override
    protected void updateDashboard() {
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
        SmartDashboard.putNumber("getRate", gyro.getRate());


        maxLeftSpeed = Math.max(leftDriveFront.getSelectedSensorVelocity(), maxLeftSpeed);
        maxRightSpeed = Math.max(rightDriveFront.getSelectedSensorVelocity(), maxRightSpeed);

        SmartDashboard.putNumber("Max Left Speed", maxLeftSpeed);
        SmartDashboard.putNumber("Max Right Speed", maxRightSpeed);

        SmartDashboard.putNumber("Left Speed", leftDriveFront.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Right Speed", rightDriveFront.getSelectedSensorVelocity());

        SmartDashboard.putNumber("Left Position", leftDriveFront.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Position", rightDriveFront.getSelectedSensorPosition());
                
        dcu.tickNarwhalDashboard();
    }

    public static void main(String... args) {
        RobotBase.startRobot(MainPrebot::new);
    }
}