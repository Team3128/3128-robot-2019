//AUTHORSES:
    //MASON HOLST
    //jude
    //caylin
    //daniel
    //tyler
    //teo

    //not adham :/


    package org.team3128.button.main;

    import com.ctre.phoenix.motorcontrol.ControlMode;
    import com.ctre.phoenix.motorcontrol.FeedbackDevice;
    import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.team3128.button.autonomous.*;
    
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
    
    import java.io.BufferedWriter;
    import java.io.FileWriter;
    import java.io.File;
    import java.io.FileInputStream;
    import java.io.FileOutputStream;
    import java.io.IOException;
    import java.io.InputStream;
    import java.io.OutputStream;
    
    
    
    public class MainButton extends NarwhalRobot {
        public SRXTankDrive drive;
        public ListenerManager lm;
        public Joystick joystick;

        public TalonSRX leftLeader;
        public TalonSRX rightLeader;
        public VictorSPX leftFollower;
        public VictorSPX rightFollower;

        private DriveCommandRunning driveCmdRunning;
        public DriveCalibrationUtility dcu;


    
        @Override
        protected void constructHardware()
        {
            leftLeader = new TalonSRX(13);
            leftFollower = new VictorSPX(5);
            rightLeader = new TalonSRX(15);
            rightFollower = new VictorSPX(6);

            leftFollower.set(ControlMode.Follower, leftLeader.getDeviceID());
            rightFollower.set(ControlMode.Follower, rightLeader.getDeviceID());

            leftLeader.setInverted(false);
            leftFollower.setInverted(false);

            rightLeader.setInverted(true);
            rightFollower.setInverted(true);

            joystick = new Joystick(1);
            lm = new ListenerManager(joystick);
            addListenerManager(lm);
            driveCmdRunning = new DriveCommandRunning();

    
            double wheelCirc = 13.21 * Length.in;
            double wheelbase = 32.3 * Length.in;
            int driveMaxSpeed = 3700;
    
            SRXTankDrive.initialize(leftLeader, rightLeader, wheelCirc, wheelbase, driveMaxSpeed);
            drive = SRXTankDrive.getInstance();
            dcu = DriveCalibrationUtility.getInstance();
        
            dcu.initNarwhalDashboard();
        }
        
        @Override
        protected void constructAutoPrograms() {
            NarwhalDashboard.addAuto("Auto Test", new CmdAutoTest());
    
        }
    
        @Override
        protected void setupListeners() {
            lm.addMultiListener(() -> {    
            if (!driveCmdRunning.isRunning) {
                drive.arcadeDrive(
                    -0.7 * RobotMath.thresh(lm.getAxis("MoveTurn"), 0.1),
                    -1.0 * RobotMath.thresh(lm.getAxis("MoveForwards"), 0.1),
                    -1.0 * lm.getAxis("Throttle"),
                     true
                );		
            }
			
        }, "MoveTurn", "MoveForwards", "Throttle");
            /*
            lm.nameControl(new Button(12), "MoveMotorForwards");
            lm.addButtonDownListener("MoveMotorForwards", () -> {
                testMotor.set(ControlMode.PercentOutput,100);
                //drive.tankDrive(1, 1);
            });
            lm.addButtonUpListener("MoveMotorForwards", () -> {
                testMotor.set(ControlMode.PercentOutput,0);
                //drive.tankDrive(0, 0);
            });
            lm.nameControl(new Button(11), "MoveMotorBackwards");
            lm.addButtonDownListener("MoveMotorBackwards", () -> {
                testMotor.set(ControlMode.PercentOutput,-100);
                //drive.tankDrive(-1, -1);
            });
            lm.addButtonUpListener("MoveMotorBackwards", () -> {
                testMotor.set(ControlMode.PercentOutput,0);
                //drive.tankDrive(0, 0);
    
            });
            */
    
    
            
            lm.nameControl(new Button(12), "leftForward");
            lm.addButtonDownListener("leftForward", () -> {
                drive.tankDrive(1, 0);
            });
            lm.addButtonUpListener("leftForward", () -> {
                drive.tankDrive(0, 0);
            });
            lm.nameControl(new Button(11), "leftBackword");
            lm.addButtonDownListener("leftBackward", () -> {
                drive.tankDrive(-1, 0);
    
            });
            lm.addButtonUpListener("leftBackward", () -> {
                drive.tankDrive(0, 0);
    
            });
            lm.nameControl(new Button(10), "rightForward");
            lm.addButtonDownListener("rightForward", () -> {
                drive.tankDrive(0, 1);
            });
            lm.addButtonUpListener("rightForward", () -> {
                drive.tankDrive(0, 0);
            });
            lm.nameControl(new Button(9), "rightBackword");
            lm.addButtonDownListener("rightBackward", () -> {
                drive.tankDrive(0, -1);
    
            });
            lm.addButtonUpListener("rightBackward", () -> {
                drive.tankDrive(0, 0);
    
            });
        }
    
        @Override
        protected void teleopPeriodic() {
            //nothing
        }
    
        @Override
        protected void updateDashboard() {
            dcu.tickNarwhalDashboard();

        }
    
    
        public static void main(String... args) {
            RobotBase.startRobot(MainButton::new);
        }
    }