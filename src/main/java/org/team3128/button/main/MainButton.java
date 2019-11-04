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

import org.team3128.athos.autonomous.*;
import org.team3128.athos.util.PrebotDeepSpaceConstants;

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

//start typing the stuff to make this a robot that isn't non-functional and bad and blank and boring and stuff thanks lol
    // - Mason Holst, "Helpful Reminders", published November 2019

public class MainButton extends NarwhalRobot {


	@Override
	protected void constructHardware()
	{

    }
    
    @Override
    protected void constructAutoPrograms() {
    }

	@Override
	protected void setupListeners() {

    }

    @Override
    protected void teleopPeriodic() {
    }

    @Override
    protected void updateDashboard() {

    }


    public static void main(String... args) {
        RobotBase.startRobot(MainButton::new);
    }
}