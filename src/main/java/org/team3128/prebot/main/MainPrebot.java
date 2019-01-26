package org.team3128.prebot.main;

import com.kauailabs.navx.frc.AHRS;


import edu.wpi.first.wpilibj.SPI;
import org.team3128.common.NarwhalRobot;
import org.team3128.prebot.autonomous.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.util.Constants;
import org.team3128.common.util.units.Length;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.CommandGroup;


public class MainPrebot extends NarwhalRobot {
    
    public AHRS ahrs;

    public TalonSRX rightDriveFront;
    public TalonSRX rightDriveMiddle;
    public TalonSRX rightDriveBack;
    public TalonSRX leftDriveFront;
    public TalonSRX leftDriveMiddle;
    public TalonSRX leftDriveBack;

    public SRXTankDrive tankDrive;

    public Joystick joystick;

    public ListenerManager lm;

    public ADXRS450_Gyro gyro;

    public double wheelCirc;
    public double wheelDiameter;

    public double maxLeftSpeed = 0;
    public double maxRightSpeed = 0;
    public NetworkTable table;
    public NetworkTable table2;

    public double valCurrent1 = 0.0;
    public double valCurrent2 = 0.0;
    public double valCurrent3 = 0.0;
    public double valCurrent4 = 0.0;

    public CommandGroup cmdRunner;

	@Override
	protected void constructHardware()
	{
		table = NetworkTableInstance.getDefault().getTable("limelight");

        rightDriveFront = new TalonSRX(0);
        rightDriveMiddle = new TalonSRX(1);
        rightDriveBack = new TalonSRX(2);

        leftDriveFront = new TalonSRX(3);
        leftDriveMiddle = new TalonSRX(4);
        leftDriveBack = new TalonSRX(5);

        rightDriveFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
        leftDriveFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
        
        rightDriveMiddle.set(ControlMode.Follower, rightDriveFront.getDeviceID());
        leftDriveMiddle.set(ControlMode.Follower, leftDriveFront.getDeviceID());
        rightDriveBack.set(ControlMode.Follower, rightDriveFront.getDeviceID());
        leftDriveBack.set(ControlMode.Follower, leftDriveFront.getDeviceID());

        wheelCirc = 12.42*Length.in;
        //wheelDiameter = 3.68 * Length.in;
        SRXTankDrive.initialize(rightDriveFront, leftDriveFront, wheelCirc, 1, 68.107, 4200);
        tankDrive = SRXTankDrive.getInstance();
        tankDrive.setLeftSpeedScalar(0.99038845);
        
        //rightDriveFront.setInverted(true);
        //rightDriveMiddle.setInverted(true);
        //rightDriveBack.setInverted(true);

        leftDriveFront.setInverted(true);
        leftDriveMiddle.setInverted(true);
        leftDriveBack.setInverted(true);

        leftDriveFront.setSensorPhase(true);
        rightDriveFront.setSensorPhase(true);
        
        joystick = new Joystick(1);
		lm = new ListenerManager(joystick);
        addListenerManager(lm);
        ahrs = new AHRS(SPI.Port.kMXP); 

        gyro = new ADXRS450_Gyro();
        gyro.calibrate();
        
        leftDriveFront.config_kP(0, 0.038);
        rightDriveFront.config_kP(3, 0.038);
    }
    
    @Override
    protected void constructAutoPrograms() {
        NarwhalDashboard.addAuto("Turn", new CmdInPlaceTurnTest());
        NarwhalDashboard.addAuto("Arc Turn", new CmdArcTurnTest());
        NarwhalDashboard.addAuto("Forward", new CmdDriveForward());
        //NarwhalDashboard.addAuto("Test", new Test(tankDrive, ahrs));
        NarwhalDashboard.addAuto("Wheel Base Test", new CmdCallibrateWheelbase(6000, 1000, 2000));
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

        lm.nameControl(new Button(11), "HalfSpeed");
		lm.addButtonDownListener("HalfSpeed", () ->
		{
			tankDrive.tankDrive(.25, .25);
		});
        lm.addButtonUpListener("HalfSpeed", () ->
		{
			tankDrive.tankDrive(0, 0);
		});

        lm.nameControl(new Button(2), "LightOn");
		lm.addButtonDownListener("LightOn", () -> {
            table.getEntry("ledMode").setNumber(3);
            Log.debug("Limelight Latency", String.valueOf(table.getEntry("tl").getDouble(0.0)));
  
        });
        /*listenerRight.nameControl(new Button(2), "LightOff");
		listenerRight.addButtonUpListener("LightOff", () -> {
		    table.getEntry("ledMode").setNumber(1);
		});*/
		lm.nameControl(ControllerExtreme3D.TRIGGER, "LogLimelight");
		lm.addButtonDownListener("LogLimelight", () -> { 
        });

        
        lm.nameControl(new Button(7), "CamMode");
        lm.addButtonDownListener("CamMode", () -> {
            for(int i = 0; i<10000; i++){
                Log.info("trigger", "trigger triggered");
                valCurrent1 = valCurrent1 + table.getEntry("tx").getDouble(0.0);
                valCurrent2 = valCurrent2 + table.getEntry("ty").getDouble(0.0);
                valCurrent3 = valCurrent3 + table.getEntry("ts").getDouble(0.0);
                valCurrent4 = valCurrent4 + table.getEntry("ta").getDouble(0.0);

            }
            valCurrent1 = valCurrent1/10000;
            valCurrent2 = valCurrent2/10000;
            valCurrent3 = valCurrent3/10000;
            valCurrent4 = valCurrent4/10000;
            Log.info("vals", String.valueOf(valCurrent1));
            NarwhalDashboard.put("txav", String.valueOf(valCurrent1));
            NarwhalDashboard.put("tyav", String.valueOf(valCurrent2));
            NarwhalDashboard.put("tzav", String.valueOf(valCurrent3));
            NarwhalDashboard.put("taav", String.valueOf(valCurrent4));
            valCurrent1 = 0.0;
            valCurrent2 = 0.0;
            valCurrent3 = 0.0;
            valCurrent4 = 0.0;
  
        });

        lm.nameControl(new Button(8), "DriveMode");
        lm.addButtonDownListener("DriveMode", () -> {
            table.getEntry("camMode").setNumber(1);
            Log.debug("Limelight Latency", String.valueOf(table.getEntry("tl").getDouble(0.0)));
  
        });

        lm.nameControl(new Button(11), "DriveLL");
        lm.addButtonDownListener("DriveLL", () -> {
            for(int i = 0; i<2000; i++){
                Log.info("trigger", "trigger triggered");
                valCurrent2 = valCurrent2 + table.getEntry("ty").getDouble(0.0);

            }
            valCurrent2 = valCurrent2/2000;

            double d = (28.5 - 9.5) / Math.tan(28.0 + valCurrent2);

            cmdRunner.addSequential(tankDrive.new CmdDriveStraight(d * Length.in, 1.0, 10000));

            Log.info("tyav", String.valueOf(valCurrent2));
            NarwhalDashboard.put("tyav", String.valueOf(valCurrent2));
            valCurrent2 = 0.0;
        });
    }
    
    @Override
    protected void updateDashboard() {
        //NarwhalDashboard.put("tx", table.getEntry("tx").getNumber(0));
        NarwhalDashboard.put("tx", table.getEntry("tx").getDouble(0.0));
        NarwhalDashboard.put("ty", table.getEntry("ty").getDouble(0.0));
        NarwhalDashboard.put("tv", table.getEntry("tv").getDouble(0.0));
        NarwhalDashboard.put("ta", table.getEntry("ta").getDouble(0.0));
        NarwhalDashboard.put("ts", table.getEntry("ts").getDouble(0.0));
        NarwhalDashboard.put("tl", table.getEntry("tl").getDouble(0.0));
        SmartDashboard.putNumber("Gyro Angle", RobotMath.normalizeAngle(gyro.getAngle()));

		SmartDashboard.putNumber("Left Speed (nu/100ms)", leftDriveFront.getSelectedSensorVelocity(0));
        SmartDashboard.putNumber("Right Speed (nu/100ms)", rightDriveFront.getSelectedSensorVelocity(0));
        
        maxLeftSpeed = Math.max(leftDriveFront.getSelectedSensorVelocity(), maxLeftSpeed);
        maxRightSpeed = Math.max(rightDriveFront.getSelectedSensorVelocity(), maxRightSpeed);

        SmartDashboard.putNumber("Max Left Speed", maxLeftSpeed);
        SmartDashboard.putNumber("Max Right Speed", maxRightSpeed);
		
    }
    public static void main(String... args) {
        RobotBase.startRobot(MainPrebot::new);
    }
}