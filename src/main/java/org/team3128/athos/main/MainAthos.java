package org.team3128.athos.main;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.team3128.athos.autonomous.*;
import org.team3128.common.generics.RobotConstants;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.drive.SRXTankDrive.Wheelbase;
import org.team3128.common.drive.calibrationutility.DriveCalibrationUtility;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.gyroscope.Gyro;
import org.team3128.common.hardware.gyroscope.NavX;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.units.Length;
import org.team3128.common.vision.CmdHorizontalOffsetFeedbackDrive;
import org.team3128.gromit.constants.DeepSpaceConstants; // Eventually move this to the common directory with organization for yearly game constants
import org.team3128.gromit.constants.GromitConstants;
import org.team3128.athos.subsystems.Constants;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.hardware.motor.LazyCANSparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import org.team3128.common.generics.ThreadScheduler;

public class MainAthos extends NarwhalRobot {
    public LazyCANSparkMax rightDriveFront, rightDriveMiddle, rightDriveBack;
    public LazyCANSparkMax leftDriveFront, leftDriveMiddle, leftDriveBack;

    public Joystick joystick;
    public ListenerManager lm;
    public Gyro gyro;

    public PIDConstants visionPID, blindPID;

    public NetworkTable table;
    public NetworkTable limelightTable;

    private DriveCommandRunning driveCmdRunning;

    public Limelight limelight = new Limelight("limelight-c", Constants.BOTTOM_LIMELIGHT_ANGLE,
            Constants.BOTTOM_LIMELIGHT_HEIGHT, Constants.BOTTOM_LIMELIGHT_DISTANCE_FROM_FRONT,
            DeepSpaceConstants.TARGET_WIDTH);

    @Override
    protected void constructHardware() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight"); // this might have to be limelight-c
                                                                                  // or whatever the hostname is?

        rightDriveFront = new LazyCANSparkMax(Constants.RIGHT_DRIVE_FRONT_ID, Constants.MOTOR_TYPE);
        rightDriveMiddle = new LazyCANSparkMax(Constants.RIGHT_DRIVE_MIDDLE_ID, Constants.MOTOR_TYPE);
        rightDriveBack = new LazyCANSparkMax(Constants.RIGHT_DRIVE_BACK_ID, Constants.MOTOR_TYPE);

        leftDriveFront = new LazyCANSparkMax(Constants.LEFT_DRIVE_FRONT_ID, Constants.MOTOR_TYPE);
        leftDriveMiddle = new LazyCANSparkMax(Constants.LEFT_DRIVE_MIDDLE_ID, Constants.MOTOR_TYPE);
        leftDriveBack = new LazyCANSparkMax(Constants.LEFT_DRIVE_BACK_ID, Constants.MOTOR_TYPE);

        double wheelCirc = Constants.WHEEL_CIRCUMFERENCE;
        double wheelBase = Constants.WHEELBASE;
        int robotFreeSpeed = Constants.DRIVE_HIGH_SPEED;

        // SRXTankDrive.initialize(leftDriveFront, rightDriveFront, wheelCirc,
        // wheelBase, robotFreeSpeed);

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
    }

    @Override
    protected void constructAutoPrograms() {
    }

    @Override
    protected void setupListeners() {
        lm.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
        lm.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
        lm.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");

        lm.addMultiListener(() -> {
            if (!driveCmdRunning.isRunning) {
                drive.arcadeDrive(-0.7 * RobotMath.thresh(lm.getAxis("MoveTurn"), 0.1),
                        -1.0 * RobotMath.thresh(lm.getAxis("MoveForwards"), 0.1), -1.0 * lm.getAxis("Throttle"), true);
            }

        }, "MoveTurn", "MoveForwards", "Throttle");

        lm.nameControl(ControllerExtreme3D.TRIGGER, "AlignToTarget");
        lm.addButtonDownListener("AlignToTarget", () -> {
            // TODO: Add current implementation of vision alignment
            Log.info("MainAthos.java", "[Vision Alignment] Not created yet, would've started");
        });
        lm.addButtonUpListener("AlignToTarget", () -> {
            Log.info("MainAthos.java", "[Vision Alignment] Not created yet, would've ended");
        });
    }

    @Override
    protected void teleopPeriodic() {
    }

    @Override
    protected void updateDashboard() {
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
        SmartDashboard.putNumber("getRate", gyro.getRate());

        // maxLeftSpeed = Math.max(leftDriveFront.getSelectedSensorVelocity(),
        // maxLeftSpeed);
        // maxRightSpeed = Math.max(rightDriveFront.getSelectedSensorVelocity(),
        // maxRightSpeed);

        // SmartDashboard.putNumber("Max Left Speed", maxLeftSpeed);
        // SmartDashboard.putNumber("Max Right Speed", maxRightSpeed);

        // SmartDashboard.putNumber("Left Speed",
        // leftDriveFront.getSelectedSensorVelocity());
        // SmartDashboard.putNumber("Right Speed",
        // rightDriveFront.getSelectedSensorVelocity());

        // SmartDashboard.putNumber("Left Position",
        // leftDriveFront.getSelectedSensorPosition());
        // SmartDashboard.putNumber("Right Position",
        // rightDriveFront.getSelectedSensorPosition());

        NarwhalDashboard.put("time", DriverStation.getInstance().getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());
    }

    public static void main(String... args) {
        RobotBase.startRobot(MainAthos::new);
    }
}