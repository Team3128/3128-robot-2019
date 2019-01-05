package org.team3128.common.drive;

import org.team3128.common.drive.routemaker.Constants;
import org.team3128.common.util.Convert;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.Log;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;

/**
 * Experimental class in order to constantly integrate encoder and gyroscope
 * readings in order to determine the displacement of the robot from its initial
 * position.
 * 
 * @author Ronak
 *
 */
public class Odometer {
	private ADXRS450_Gyro gyro;
	private TalonSRX leftDriveMotors, rightDriveMotors;

	/**
	 * The circumference of the wheels, in centimeters
	 */
	private double wheelCirc;

	/**
	 * The x and y displacement of the robot with respect to the last reset, in
	 * centimeters.
	 */
	private double xPosition, yPosition;

	/**
	 * The gyro return angle that we should assume to be zero, since gyroscopes
	 * can't reset with the robot on.
	 */
	private double zeroAngle;

	public static Odometer getInstance() {
		if (instance != null) {
			return instance;
		}
		
		Log.fatal("Odometer", "Attempted to get instance before initializtion! Call initialize(...) first.");
		return null;
	}

	private static Odometer instance = null;
	public static void initialize(ADXRS450_Gyro gyro, double x, double y, double angle) {
		instance = new Odometer(gyro, x, y, angle);
	}

	private Odometer(ADXRS450_Gyro gyro, double x, double y, double angle) {
		SRXTankDrive drive = SRXTankDrive.getInstance();

		this.leftDriveMotors = drive.getLeftMotors();
		this.rightDriveMotors = drive.getRightMotors();

		this.wheelCirc = drive.wheelCircumfrence;

		this.gyro = gyro;

		setPosition(x, y, angle);

		oldLeft = leftDriveMotors.getSelectedSensorPosition(0);
		oldRight = leftDriveMotors.getSelectedSensorPosition(0);

		oldTheta = getAngle();

		lastUpdateTime = Timer.getFPGATimestamp();
	}

	private double lastUpdateTime;

	private double oldLeft, oldRight;

	private double left, right;
	private double dLeft, dRight;

	private double oldTheta;

	private double theta;
	private double dTheta;

	private double r;

	public void update() {
		left = leftDriveMotors.getSelectedSensorPosition(0);
		right = rightDriveMotors.getSelectedSensorPosition(0);

		theta = gyro.getAngle();

		lastUpdateTime = Timer.getFPGATimestamp();

		dLeft = left - oldLeft;
		dRight = right - oldRight;

		dTheta = theta - oldTheta;
		if (dTheta > 360 - Constants.ARBITRARY_LOOP_PROTECTION_BUFFER) {
			dTheta = 360 - dTheta;
		}
		else if (dTheta < Constants.ARBITRARY_LOOP_PROTECTION_BUFFER - 360) {
			dTheta = 360 + dTheta;
		}

		r = Convert.lengthCTREtoCM(dLeft + dRight, wheelCirc) / (2 * Math.toRadians(dTheta));

		incrementPosition(
			r * (RobotMath.cos(dTheta) - 1.0),
			r * RobotMath.sin(dTheta)
		);

		oldLeft = left;
		oldRight = right;
		oldTheta = theta;
	}

	public void setPosition(double x, double y, double theta) {
		xPosition = x;
		yPosition = y;

		zeroAngle = gyro.getAngle() - theta;
	}

	private synchronized void incrementPosition(double dx, double dy) {
		xPosition += dx;
		yPosition += dy;
	}

	public double getX() {
		return xPosition;
	}

	public double getY() {
		return yPosition;
	}

	public double getAngle() {
		return gyro.getAngle() - zeroAngle;
	}

	public double getLastUpdateTime() {
		return lastUpdateTime;
	}
}
