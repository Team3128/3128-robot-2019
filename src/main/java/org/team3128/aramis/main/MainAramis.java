//formerly wallace ;)

package org.team3128.aramis.main;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.team3128.aramis.autonomous.*;
import org.team3128.aramis.util.PrebotDeepSpaceConstants;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.drive.SRXTankDrive.Wheelbase;
import org.team3128.common.drive.calibrationutility.DriveCalibrationUtility;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.navigation.Gyro;
import org.team3128.common.hardware.navigation.NavX;
import org.team3128.common.util.Constants;
import org.team3128.common.util.units.Angle;
import org.team3128.common.util.units.Length;
import org.team3128.common.vision.CmdHorizontalOffsetFeedbackDrive;
import org.team3128.gromit.util.DeepSpaceConstants;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.datatypes.PIDConstants;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class MainAramis extends NarwhalRobot {
    public TalonSRX rightDriveLeader;
    public VictorSPX rightDriveFollower;
    public TalonSRX leftDriveLeader;
    public VictorSPX leftDriveFollower;

    public SRXTankDrive tankDrive;

    public Joystick joystick;
    public ListenerManager lm;

    public Gyro gyro;

    public PIDConstants leftMotionProfilePID, rightMotionProfilePID;

    public double speedScalar;

    public double maxLeftSpeed = 0;
    public double maxRightSpeed = 0;
    public double leftSpeed = 0;
    public double rightSpeed = 0;
    public NetworkTable table;
    public NetworkTable table2;

    PIDConstants visionPID, blindPID;

    public NetworkTable limelightTable;

    public DriveCalibrationUtility dcu;
    public Wheelbase calculatedWheelbase;

    public CmdHorizontalOffsetFeedbackDrive alignCommand;
    private DriveCommandRunning driveCmdRunning;

    public Limelight limelight = new Limelight("limelight", 26 * Angle.DEGREES, 6.15 * Length.in, 0 * Length.in, 14.5 * Length.in);

	@Override
	protected void constructHardware()
	{
		limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

        rightDriveLeader = new TalonSRX(10);
        rightDriveFollower = new VictorSPX(11);

        leftDriveLeader = new TalonSRX(15);
        leftDriveFollower = new VictorSPX(16);

        rightDriveLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
        rightDriveFollower.set(ControlMode.Follower, rightDriveLeader.getDeviceID());

        leftDriveLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
        leftDriveFollower.set(ControlMode.Follower, leftDriveLeader.getDeviceID());

        double wheelCirc = 13.21 * Length.in;
        //double wheelBase = 31.5 * Length.in; 5 feet
        //double wheelBase = 32.25 * Length.in; 4 feet
        double wheelBase = 32.3 * Length.in;
        int robotFreeSpeed = 3700;

        SRXTankDrive.initialize(leftDriveLeader, rightDriveLeader, wheelCirc, wheelBase, robotFreeSpeed);

        leftDriveLeader.setInverted(false);
        leftDriveFollower.setInverted(false);

        rightDriveLeader.setInverted(true);
        rightDriveFollower.setInverted(true);

        leftDriveLeader.setSensorPhase(true);
        rightDriveLeader.setSensorPhase(true);

        tankDrive = SRXTankDrive.getInstance();
        tankDrive.setLeftSpeedScalar(1.0);
        tankDrive.setRightSpeedScalar(.983);
        
        // Instatiator if we're using the NavX
		gyro = new NavX();

		// Instatiator if we're using the KoP Gyro
		// gyro = new AnalogDevicesGyro();
		// ((AnalogDevicesGyro) gyro).recalibrate();

        joystick = new Joystick(1);
		lm = new ListenerManager(joystick);
        addListenerManager(lm);

        // Vision
        visionPID = new PIDConstants(0, 0.02, 0.0, 0.00001);
        blindPID = new PIDConstants(0.1, 0, 0, 0);
        driveCmdRunning = new DriveCommandRunning();

        // DCU
		DriveCalibrationUtility.initialize(gyro, visionPID);
        dcu = DriveCalibrationUtility.getInstance();
        

        dcu.initNarwhalDashboard();
    }
    
    @Override
    protected void constructAutoPrograms() {
        NarwhalDashboard.addAuto("Turn", new CmdInPlaceTurnTest());

        // NarwhalDashboard.addAuto("Arc, 5ft, Left",  tankDrive.new CmdArcTurn(5 * Length.ft, 90, Direction.LEFT, .75, 10000));
        // NarwhalDashboard.addAuto("Arc, 4ft, Left",  tankDrive.new CmdArcTurn(4 * Length.ft, 90, Direction.LEFT, .75, 10000));
        // NarwhalDashboard.addAuto("Arc, 3ft, Left",  tankDrive.new CmdArcTurn(3 * Length.ft, 90, Direction.LEFT, .75, 10000));

        // NarwhalDashboard.addAuto("Arc, 5ft, Right", tankDrive.new CmdArcTurn(5 * Length.ft, 90, Direction.RIGHT, .75, 10000));
        // NarwhalDashboard.addAuto("Arc, 4ft, Right", tankDrive.new CmdArcTurn(5 * Length.ft, 90, Direction.RIGHT, .75, 10000));
        // NarwhalDashboard.addAuto("Arc, 3ft, Right", tankDrive.new CmdArcTurn(5 * Length.ft, 90, Direction.RIGHT, .75, 10000));

        // NarwhalDashboard.addAuto("In-Place, 180, Left", tankDrive.new CmdInPlaceTurn(180 * Angle.DEGREES, Direction.LEFT, 0.75, 5000));

        NarwhalDashboard.addAuto("Please Work", new CmdPleaseWorkTurnTest(gyro));

        NarwhalDashboard.addAuto("Forward", new CmdDriveForward());
        //NarwhalDashboard.addAuto("Test", new Test(tankDrive, ahrs));
        // NarwhalDashboard.addAuto("Wheel Base Test", new CmdCalibrateWheelbase(ahrs, 10, 1000, 1500, calculatedWheelbase));
        // NarwhalDashboard.addAuto("Forward CV", new CmdDriveForwardCVTest());
        NarwhalDashboard.addAuto("Routemaker Test", new CmdRoutemakerTest());
        // NarwhalDashboard.addAuto("Heading Then Arc Turn", new CmdHeadingThenArc(limelight));
        NarwhalDashboard.addAuto("Fancy Wheel Base Calibration", new CmdFancyCalibrateWheelBase(gyro));
        NarwhalDashboard.addAuto("CmdDynamicAdjsut", new CmdDynamicAdjustTest(gyro, limelight));
        // previous speeds that were used were 2000, 4000 (arbitrarily picked)
    }

	@Override
	protected void setupListeners() {
        lm.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
		lm.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
		lm.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");		

        lm.addMultiListener(() -> {
            if (!driveCmdRunning.isRunning) {
                tankDrive.arcadeDrive(
                    -0.7 * RobotMath.thresh(lm.getAxis("MoveTurn"), 0.1),
                    -1.0 * RobotMath.thresh(lm.getAxis("MoveForwards"), 0.1),
                    -1.0 * lm.getAxis("Throttle"),
                     true
                );		
            }
			
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

		lm.nameControl(ControllerExtreme3D.TRIGGER, "AlignToTarget");
		lm.addButtonDownListener("AlignToTarget", () -> { 
            // alignCommand = new CmdHorizontalOffsetFeedbackDrive(gyro, limelight, visionPID, driveCmdRunning,
            // -1 * Angle.DEGREES, 14.5 * Length.in, DeepSpaceConstants.DECELERATE_START_DISTANCE, DeepSpaceConstants.DECELERATE_END_DISTANCE,
            // blindPID, false);
            // alignCommand.start();
        });
        lm.addButtonUpListener("AlignToTarget", () -> {
            alignCommand.cancel();
            alignCommand = null;
        });

        lm.nameControl(new Button(3), "GetValues");
        lm.addButtonDownListener("GetValues", () -> {
            //thing.write(limelight.getValues(30).toString());
        });
    }

    @Override
    protected void teleopPeriodic() {
    }

    @Override
    protected void updateDashboard() {
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
        SmartDashboard.putNumber("getRate", gyro.getRate());


        maxLeftSpeed = Math.max(leftDriveLeader.getSelectedSensorVelocity(), maxLeftSpeed);
        maxRightSpeed = Math.max(rightDriveLeader.getSelectedSensorVelocity(), maxRightSpeed);

        SmartDashboard.putNumber("Max Left Speed", maxLeftSpeed);
        SmartDashboard.putNumber("Max Right Speed", maxRightSpeed);

        SmartDashboard.putNumber("Left Speed", leftDriveLeader.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Right Speed", rightDriveLeader.getSelectedSensorVelocity());

        SmartDashboard.putNumber("Left Position", leftDriveLeader.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Position", rightDriveLeader.getSelectedSensorPosition());

        NarwhalDashboard.put("time", DriverStation.getInstance().getMatchTime());
		NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());
                
        dcu.tickNarwhalDashboard();

        // USB.write(limelight.getValues(30).toString());

    }

    public static void main(String... args) {
        RobotBase.startRobot(MainAramis::new);
    }
}