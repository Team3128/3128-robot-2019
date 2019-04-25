package org.team3128.common.drive.calibrationutility;

import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.hardware.navigation.Gyro;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.util.Log;
import org.team3128.common.util.datatypes.PIDConstants;

public class DriveCalibrationUtility {
    private static DriveCalibrationUtility instance = null;
	public static DriveCalibrationUtility getInstance() {
		if (instance != null) {
			return instance;
		}

		Log.fatal("DriveCalibrationUtility", "Attempted to get instance before initializtion! Call initialize(...) first.");
		return null;
    }
    
    public static void initialize(Gyro gyro, PIDConstants visionPID) {
        instance = new DriveCalibrationUtility(gyro, visionPID);
    }

    public double maxLeftSpeed = 0;
    public double maxRightSpeed = 0;
	    
    public SRXTankDrive drive;
    public Gyro gyro;

    public PIDConstants visionPID;

    // private double wheelbaseSum;
    // private int wheelbaseCount;
    // private FeedForwardPowerMultiplierSet ffpmSet;
    // private WheelbaseSet wbSet;

    private DriveCalibrationUtility(Gyro gyro, PIDConstants visionPID) {
        maxLeftSpeed = 0;
        maxRightSpeed = 0;
        this.gyro = gyro;

        drive = SRXTankDrive.getInstance();
    }

    public void initNarwhalDashboard() {
        NarwhalDashboard.addButton("resetEncoders", (boolean down) -> {
            if (down) {
                drive.getLeftMotors().setSelectedSensorPosition(0);
                drive.getRightMotors().setSelectedSensorPosition(0);
            }
        });

        NarwhalDashboard.addButton("resetMaxSpeed", (boolean down) -> {
            if (down) {
                maxLeftSpeed = 0;
                maxRightSpeed = 0;
            }
        });

        //NarwhalDashboard.put("vision_p", visionPID.kP);
		//NarwhalDashboard.put("vision_i", visionPID.kI);
        //NarwhalDashboard.put("vision_d", visionPID.kD);
        
        NarwhalDashboard.addNumDataListener("visionPID", (double constants[]) ->{
            visionPID.kP = constants[0];
            visionPID.kI = constants[1];
            visionPID.kD = constants[2];
        });

        NarwhalDashboard.addNumDataListener("drivePID", (double constants[]) -> {
			//this.leftMotionProfilePID.kF = constants[0];
			drive.leftMotionProfilePID.kP = constants[0];
			drive.leftMotionProfilePID.kI = constants[1];
			drive.leftMotionProfilePID.kD = constants[2];

			//this.rightMotionProfilePID.kF = constants[0];
			drive.rightMotionProfilePID.kP = constants[3];
			drive.rightMotionProfilePID.kI = constants[4];
			drive.rightMotionProfilePID.kD = constants[5];

			// this.leftVelocityPID.kF = constants[0];
			// this.leftVelocityPID.kP = constants[4];
			// this.leftVelocityPID.kI = constants[5];
			// this.leftVelocityPID.kD = constants[6];

			// this.rightVelocityPID.kF = constants[0];
			// this.rightVelocityPID.kP = constants[4];
			// this.rightVelocityPID.kI = constants[5];
			// this.rightVelocityPID.kD = constants[6];

            drive.setPID();
            sendDrivePIDConstants();
        });
        sendDrivePIDConstants();

        // NarwhalDashboard.addNumDataListener("calc_wb", (double[] data) -> {
        //     Log.info("test", "triggered");
        //     double pL = data[0];
        //     double pR = data[1];

        //     int duration = (int) data[2];
        //     wbSet = new WheelbaseSet();
            
        //     drive.new CmdCalculateWheelbase(wbSet, pL, pR, gyro, duration).start();
        // });

        // NarwhalDashboard.addButton("printCSV_wb", (boolean down) -> {
        //     if (down) {
        //         Log.info("DriveCalibrationUtility", "average:\n" + wbSet.getAvgCSV());
		// 	    Log.info("DriveCalibrationUtility", "data:\n" + wbSet.getAllCSV());
        //     }
        // });

        // NarwhalDashboard.addButton("reset_wb_avg", (boolean down) -> {
        //     if (down) {
        //         wheelbaseSum = 0;
        //         wheelbaseCount = 0;
        //     }
        // });
        // NarwhalDashboard.addNumDataListener("calc_ffp", (double[] data) -> {
        //     double pL = data[0];
        //     double pR = data[1];

        //     int duration = (int) data[2];

        //     ffpmSet = new FeedForwardPowerMultiplierSet();
        //     drive.new CmdGetFeedForwardPowerMultiplier(ffpmSet,gyro,pL,pR,duration).start();
        // });

        // NarwhalDashboard.addButton("printCSV", (boolean down) -> {
        //     if (down) {
        //         Log.info("DriveCalibrationUtility", "average:\n" + ffpmSet.getAvgCSV());
		// 	    Log.info("DriveCalibrationUtility", "data:\n" + ffpmSet.getAllCSV());
        //     }
        // });
        
        // NarwhalDashboard.addButton("pidCalDrive", (boolean down) -> {
        //     if (down) {
        //         new Cmd100InchDrive().start();
        //     }
        // });
    }

    public void tickNarwhalDashboard() {
        maxLeftSpeed = Math.max(maxLeftSpeed, Math.abs(drive.getLeftMotors().getSelectedSensorVelocity()));
        maxRightSpeed = Math.max(maxRightSpeed, Math.abs(drive.getRightMotors().getSelectedSensorVelocity()));

        NarwhalDashboard.put("wheel_circumference", getWheelCirc());

        NarwhalDashboard.put("left_max_speed", maxLeftSpeed);
        NarwhalDashboard.put("right_max_speed", maxRightSpeed);

        NarwhalDashboard.put("left_scalar", getLeftSpeedScalar());
        NarwhalDashboard.put("right_scalar", getRightSpeedScalar());

        // NarwhalDashboard.put("prev_wb", calculatedWheelbase.wheelbase / Length.in);
        // NarwhalDashboard.put("avg_wb", wheelbaseSum / (wheelbaseCount * Length.in));

        // NarwhalDashboard.put("avg_w", ffpmSet.ffpmAvg.angularVelocity);
        // NarwhalDashboard.put("avg_ffpmL", ffpmSet.ffpmAvg.ffpL);
        // NarwhalDashboard.put("avg_ffpmR", ffpmSet.ffpmAvg.ffpR);
    }

    public double getWheelCirc() {
        if (drive.getLeftMotors().getSelectedSensorPosition() == 0 || drive.getRightMotors().getSelectedSensorPosition() == 0) {
            return -1;
        }

        double averagePosition = (drive.getLeftMotors().getSelectedSensorPosition() + drive.getRightMotors().getSelectedSensorPosition()) / 2;

        return 100 * averagePosition/4096;
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

	/**
	 * Sends PID constants to NarwhalDashboard
	 */
	public void sendDrivePIDConstants() {
		NarwhalDashboard.put("l_f", drive.leftMotionProfilePID.kF);

		NarwhalDashboard.put("l_mp_p", drive.leftMotionProfilePID.kP);
		NarwhalDashboard.put("l_mp_i", drive.leftMotionProfilePID.kI);
		NarwhalDashboard.put("l_mp_d", drive.leftMotionProfilePID.kD);

		// NarwhalDashboard.put("l_v_p", leftVelocityPID.kP);
		// NarwhalDashboard.put("l_v_i", leftVelocityPID.kI);
		// NarwhalDashboard.put("l_v_d", leftVelocityPID.kD);


		NarwhalDashboard.put("r_f", drive.rightMotionProfilePID.kF);

		NarwhalDashboard.put("r_mp_p", drive.rightMotionProfilePID.kP);
		NarwhalDashboard.put("r_mp_i", drive.rightMotionProfilePID.kI);
		NarwhalDashboard.put("r_mp_d", drive.rightMotionProfilePID.kD);

		// NarwhalDashboard.put("r_v_p", leftVelocityPID.kP);
		// NarwhalDashboard.put("r_v_i", leftVelocityPID.kI);
		// NarwhalDashboard.put("r_v_d", leftVelocityPID.kD);
    }
}