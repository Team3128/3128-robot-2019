package org.team3128.prebot.main;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team3128.prebot.autonomous.*;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.util.Constants;
import org.team3128.common.util.units.Length;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.Wheelbase;
import org.team3128.common.util.datatypes.PIDConstants;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;

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

    public ADXRS450_Gyro gyro;
    public AHRS ahrs;

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

    public Wheelbase calculatedWheelbase;

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

        double wheelCirc = 12.42 * Length.in;
        double wheelBase = 68.107 * Length.in;
        int robotFreeSpeed = 4200;

        SRXTankDrive.initialize(rightDriveFront, leftDriveFront, wheelCirc, 1.0, wheelBase, robotFreeSpeed);
        tankDrive = SRXTankDrive.getInstance();

        tankDrive.setLeftSpeedScalar(1.0);
        tankDrive.setRightSpeedScalar(0.99319568);

        leftDriveFront.setInverted(true);
        leftDriveMiddle.setInverted(true);
        leftDriveBack.setInverted(true);

        leftDriveFront.setSensorPhase(true);
        rightDriveFront.setSensorPhase(true);
        
        ahrs = new AHRS(SPI.Port.kMXP); 

        gyro = new ADXRS450_Gyro();
        gyro.calibrate();

        leftMotionProfilePID = new PIDConstants(0.253, 0.038, 0, 0);
        leftVelocityPID = new PIDConstants(0.253, 0, 0, 0);

        rightMotionProfilePID = new PIDConstants(0.253, 0.038, 0, 0);
        rightVelocityPID = new PIDConstants(0.253, 0, 0, 0);

        tankDrive.configurePID(leftMotionProfilePID, leftVelocityPID, rightMotionProfilePID, rightVelocityPID);

        joystick = new Joystick(1);
		lm = new ListenerManager(joystick);
        addListenerManager(lm);
        
        NarwhalDashboard.addButton("resetEncoders", (boolean down) -> {
            if (down) {
                tankDrive.getLeftMotors().setSelectedSensorPosition(0);
                tankDrive.getRightMotors().setSelectedSensorPosition(0);
            }
        });

        NarwhalDashboard.addButton("resetMaxSpeed", (boolean down) -> {
            if (down) {
                maxLeftSpeed = 0;
                maxRightSpeed = 0;
            }
        });

        calculatedWheelbase = new Wheelbase();
        NarwhalDashboard.addButton("wheelbase", (boolean down) -> {
            if (down) {
                new CmdCallibrateWheelbase(ahrs, 10, 1000, 1500, calculatedWheelbase);
            }
        });
        
        NarwhalDashboard.addButton("pidCalDrive", (boolean down) -> {
            if (down) {
                new CmdDriveForward().start();
            }
        });
    }
    
    @Override
    protected void constructAutoPrograms() {
        NarwhalDashboard.addAuto("Turn", new CmdInPlaceTurnTest());
        NarwhalDashboard.addAuto("Arc Turn", new CmdArcTurnTest());
        NarwhalDashboard.addAuto("Forward", new CmdDriveForward());
        //NarwhalDashboard.addAuto("Test", new Test(tankDrive, ahrs));
        NarwhalDashboard.addAuto("Wheel Base Test", new CmdCallibrateWheelbase(ahrs, 10, 1000, 1500, calculatedWheelbase));
        NarwhalDashboard.addAuto("Forward CV", new CmdDriveForwardCVTest());
        NarwhalDashboard.addAuto("Routemaker Test", new CmdRoutemakerTest());
        // previous speeds that were used were 2000, 4000 (arbitrarily picked)
    }

	@Override
	protected void setupListeners() {
        lm.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
		lm.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
		lm.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");		

        lm.addMultiListener(() -> {
			tankDrive.arcadeDrive(-0.3 * lm.getAxis("MoveTurn"),
					lm.getAxis("MoveForwards"),
					-1 * lm.getAxis("Throttle"),
					true);		
        }, "MoveTurn", "MoveForwards", "Throttle");

        lm.nameControl(new Button(12), "FullSpeed");
        lm.addButtonDownListener("FullSpeed", () ->
		{
			tankDrive.tankDrive(1, 1);
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

        
        // lm.nameControl(new Button(7), "CamMode");
        // lm.addButtonDownListener("CamMode", () -> {
        //     for(int i = 0; i<10000; i++){
        //         Log.info("trigger", "trigger triggered");
        //         valCurrent1 = valCurrent1 + limelightTable.getEntry("tx").getDouble(0.0);
        //         valCurrent2 = valCurrent2 + limelightTable.getEntry("ty").getDouble(0.0);
        //         valCurrent3 = valCurrent3 + limelightTable.getEntry("ts").getDouble(0.0);
        //         valCurrent4 = valCurrent4 + limelightTable.getEntry("ta").getDouble(0.0);

        //     }
        //     valCurrent1 = valCurrent1/10000;
        //     valCurrent2 = valCurrent2/10000;
        //     valCurrent3 = valCurrent3/10000;
        //     valCurrent4 = valCurrent4/10000;
        //     Log.info("vals", String.valueOf(valCurrent1));
        //     NarwhalDashboard.put("txav", String.valueOf(valCurrent1));
        //     NarwhalDashboard.put("tyav", String.valueOf(valCurrent2));
        //     NarwhalDashboard.put("tzav", String.valueOf(valCurrent3));
        //     NarwhalDashboard.put("taav", String.valueOf(valCurrent4));
        //     valCurrent1 = 0.0;
        //     valCurrent2 = 0.0;
        //     valCurrent3 = 0.0;
        //     valCurrent4 = 0.0;
  
        // });

        // lm.nameControl(new Button(8), "DriveMode");
        // lm.addButtonDownListener("DriveMode", () -> {
        //     limelightTable.getEntry("camMode").setNumber(1);
        //     Log.debug("Limelight Latency", String.valueOf(limelightTable.getEntry("tl").getDouble(0.0)));
  
        // });

        // lm.nameControl(new Button(11), "DriveLL");
        // lm.addButtonDownListener("DriveLL", () -> {
        //     for(int i = 0; i<2000; i++){
        //         Log.info("trigger", "trigger triggered");
        //         valCurrent2 = valCurrent2 + limelightTable.getEntry("ty").getDouble(0.0);

        //     }
        //     valCurrent2 = valCurrent2/2000;

        //     double d = (28.5 - 9.5) / Math.tan(28.0 + valCurrent2);

        //     (tankDrive.new CmdDriveStraight(d * Length.in, 1.0, 10000)).start();

        //     Log.info("tyav", String.valueOf(valCurrent2));
        //     NarwhalDashboard.put("tyav", String.valueOf(valCurrent2));
        //     valCurrent2 = 0.0;
        // });
    }
    
    @Override
    protected void updateDashboard() {
        //NarwhalDashboard.put("tx", table.getEntry("tx").getNumber(0));
        NarwhalDashboard.put("tx", limelightTable.getEntry("tx").getDouble(0.0));
        NarwhalDashboard.put("ty", limelightTable.getEntry("ty").getDouble(0.0));
        NarwhalDashboard.put("tv", limelightTable.getEntry("tv").getDouble(0.0));
        NarwhalDashboard.put("ta", limelightTable.getEntry("ta").getDouble(0.0));
        NarwhalDashboard.put("ts", limelightTable.getEntry("ts").getDouble(0.0));
        NarwhalDashboard.put("tl", limelightTable.getEntry("tl").getDouble(0.0));

        NarwhalDashboard.put("wheelCirc", this.getWheelCirc());
        NarwhalDashboard.put("leftKf", this.getLeftKf());
        NarwhalDashboard.put("rightKf", this.getRightKf());
        NarwhalDashboard.put("leftSpeedScalar", this.getLeftSpeedScalar());
        NarwhalDashboard.put("rightSpeedScalar", this.getRightSpeedScalar());
        NarwhalDashboard.put("wheelBase", calculatedWheelbase.wheelbase);
        NarwhalDashboard.put("leftVelocityError", calculatedWheelbase.leftVelocityError);
        NarwhalDashboard.put("rightVelocityError", calculatedWheelbase.rightVelocityError);

        SmartDashboard.putNumber("Gyro Angle", RobotMath.normalizeAngle(gyro.getAngle()));

        maxLeftSpeed = leftDriveFront.getSelectedSensorVelocity();
        maxRightSpeed = rightDriveFront.getSelectedSensorVelocity();

        SmartDashboard.putNumber("Left Speed", maxLeftSpeed);
        SmartDashboard.putNumber("Right Speed", maxRightSpeed);

        //leftSpeed = leftDriveFront.getSelectedSensorVelocity();
        //rightSpeed = rightDriveFront.getSelectedSensorVelocity();

        //SmartDashboard.putNumber("Left Speed (nu/100ms)", leftSpeed);
        //SmartDashboard.putNumber("Right Speed (nu/100ms)", rightSpeed);
        		
    }

    public double getWheelCirc() {
        if (leftDriveFront.getSelectedSensorPosition() == 0 || rightDriveFront.getSelectedSensorPosition() == 0) {
            return -1;
        }

        double averagePosition = (leftDriveFront.getSelectedSensorPosition() + rightDriveFront.getSelectedSensorPosition()) / 2;

        return 100 * 4096 / averagePosition;
    }
    public double getLeftKf() {
        if (maxLeftSpeed != 0) {
            return 1023 / maxLeftSpeed;
        }
        else {
            return -1;
        }
    }
    public double getRightKf() {
        if (maxLeftSpeed != 0) {
            return 1023 / maxRightSpeed;
        }
        else {
            return -1;
        }
    }
    public double getLeftSpeedScalar() {
        if (maxLeftSpeed < maxRightSpeed) {
            return maxLeftSpeed/maxRightSpeed;
        }
        else {
            return 1.0;
        }
    }
    public double getRightSpeedScalar() {
        if (maxRightSpeed < maxLeftSpeed) {
            return maxRightSpeed/maxLeftSpeed;
        }
        else {
            return 1.0;
        }
    }

    public static void main(String... args) {
        RobotBase.startRobot(MainPrebot::new);
    }
}