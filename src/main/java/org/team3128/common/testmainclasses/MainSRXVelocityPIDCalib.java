package org.team3128.common.testmainclasses;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.util.Log;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MainSRXVelocityPIDCalib extends NarwhalRobot 
{
	
	//change these to match your robot
	static int CAN_ID = 0;
	static double TESTING_MOTOR_POWER = 1;
	
	private enum State
	{
		TEST_DIR,
		CALIB_FEEDFORWARD,
		CALIB_PID
	}
	
	final static String TAG = "SRX Velocity Calibrator";
	
	TalonSRX testSRX;
	
	private State state = State.TEST_DIR;
	
	private Joystick joy;
	
	// estimated max speed of the motor
	private double motorMaxRPM;
		
	@Override
	protected void constructHardware() 
	{
		testSRX = new TalonSRX(CAN_ID);
		testSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 500);
		testSRX.config_kP(0, 0, 500);
		testSRX.config_kI(0, 0, 500);
		testSRX.config_kD(0, 0, 500);
		testSRX.config_kF(0, 0, 500);
		testSRX.config_IntegralZone(0, 0, 500);
		testSRX.configClosedloopRamp(1, 500);
		
		testSRX.setSensorPhase(true);
		
		joy = new Joystick(0);
		
		Log.info(TAG, "Started");
		Log.info(TAG, "Testing that encoder is not reversed");
	}

	@Override
	protected void setupListeners() {
		// TODO Auto-generated method stub

	}

	@Override
	protected void teleopInit() {
		testSRX.set(ControlMode.PercentOutput, TESTING_MOTOR_POWER);

	}

	@Override
	protected void autonomousInit() {
		// TODO Auto-generated method stub

	}
	
	@Override
	protected void teleopPeriodic()
	{
		switch(state)
		{
		case TEST_DIR:
			Log.info(TAG, "speed: " + testSRX.getSelectedSensorVelocity(0));
			if(testSRX.getSelectedSensorVelocity(0) > 0)
			{
				state = State.CALIB_FEEDFORWARD;
				Log.info(TAG, "Encoder setup correct.  Calibrating Feedforward...");
				
			}
			else if(testSRX.getSelectedSensorVelocity(0) < 0)
			{
				Log.recoverable(TAG, "The encoder is backwards!  Please reverse it and try again");
				testSRX.neutralOutput();
			}
			break;
		case CALIB_FEEDFORWARD:
			double velocityNativeUnitsPer100ms = testSRX.getSelectedSensorVelocity(0);
			double feedforward = (1023 * TESTING_MOTOR_POWER) / velocityNativeUnitsPer100ms; 
			Log.info(TAG, "Feedforward: " + feedforward + " (Press Trigger to use this value and continue)");
			
			if(joy.getTrigger())
			{
				motorMaxRPM = testSRX.getSelectedSensorVelocity(0);
				SmartDashboard.putNumber("P", 0);
				SmartDashboard.putNumber("I", 0);
				SmartDashboard.putNumber("D", 0);
				
				testSRX.config_kF(0, feedforward, 500);
				state = State.CALIB_PID;
								
				Log.info(TAG, "Please calibrate PID constants. The joystick controls speed.");
				Log.info(TAG, "When these PID values work, you're done!");

			}
			break;
		case CALIB_PID:
			double targetRPM = motorMaxRPM * joy.getY();
			
			SmartDashboard.putNumber("Target RPM: ", targetRPM);
			SmartDashboard.putNumber("Error (RPM)", testSRX.getClosedLoopError(0));
			
			testSRX.set(ControlMode.Velocity, targetRPM);
			
			testSRX.config_kP(0, SmartDashboard.getNumber("P", 0), 500);
			testSRX.config_kI(0, SmartDashboard.getNumber("I", 0), 500);
			testSRX.config_kD(0, SmartDashboard.getNumber("D", 0), 500);
		}
	}
	
	@Override
	protected void updateDashboard()
	{
		SmartDashboard.putNumber("Speed (RPM)", testSRX.getSelectedSensorVelocity(0));
	}

}
