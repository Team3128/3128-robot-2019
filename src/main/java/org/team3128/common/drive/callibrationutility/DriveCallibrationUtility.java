package org.team3128.common.drive.callibrationutility;

import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.drive.SRXTankDrive.Wheelbase;
import org.team3128.common.hardware.navigation.Gyro;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.util.Log;

public class DriveCallibrationUtility {
    private static DriveCallibrationUtility instance = null;
	public static DriveCallibrationUtility getInstance() {
		if (instance != null) {
			return instance;
		}

		Log.fatal("DriveCallibrationUtility", "Attempted to get instance before initializtion! Call initialize(...) first.");
		return null;
    }
    
    public static void initialize(Gyro gyro) {
        instance = new DriveCallibrationUtility(gyro);
    }

    public double maxLeftSpeed = 0;
    public double maxRightSpeed = 0;
	
    public Wheelbase calculatedWheelbase;
    
    public SRXTankDrive drive;
    public Gyro gyro;

    private double wheelbaseSum;
    private int wheelbaseCount;

    private DriveCallibrationUtility(Gyro gyro) {
        maxLeftSpeed = 0;
        maxRightSpeed = 0;

        calculatedWheelbase = new Wheelbase();
        this.gyro = gyro;

        drive = SRXTankDrive.getInstance();
        calculatedWheelbase = new Wheelbase();
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

        NarwhalDashboard.addNumDataListener("calc_wb", (double[] data) -> {
            double pL = data[0];
            double pR = data[1];

            int duration = (int) data[2];

            drive.new CmdCalculateWheelbase(calculatedWheelbase, pL, pR, gyro, duration).start();
        });

        NarwhalDashboard.addButton("add_wb_to_avg", (boolean down) -> {
            if (down) {
                wheelbaseSum += calculatedWheelbase.wheelbase;
                wheelbaseCount += 1;
            }
        });

        NarwhalDashboard.addButton("reset_wb_avg", (boolean down) -> {
            if (down) {
                wheelbaseSum = 0;
                wheelbaseCount = 0;
            }
        });
        
        NarwhalDashboard.addButton("pidCalDrive", (boolean down) -> {
            if (down) {
                new Cmd100InchDrive().start();
            }
        });
    }

    public void tickNarwhalDashboard() {
        maxLeftSpeed = Math.max(maxLeftSpeed, Math.abs(drive.getLeftMotors().getSelectedSensorVelocity()));
        maxRightSpeed = Math.max(maxRightSpeed, Math.abs(drive.getRightMotors().getSelectedSensorVelocity()));

        NarwhalDashboard.put("wheelCirc", getWheelCirc());

        NarwhalDashboard.put("left_max_speed", maxLeftSpeed);
        NarwhalDashboard.put("right_max_speed", maxRightSpeed);

        NarwhalDashboard.put("leftSpeedScalar", getLeftSpeedScalar());
        NarwhalDashboard.put("rightSpeedScalar", getRightSpeedScalar());

        NarwhalDashboard.put("prev_wb", calculatedWheelbase.wheelbase);
        NarwhalDashboard.put("avg_wb", wheelbaseSum / wheelbaseCount);

    }

    public double getWheelCirc() {
        if (drive.getLeftMotors().getSelectedSensorPosition() == 0 || drive.getRightMotors().getSelectedSensorPosition() == 0) {
            return -1;
        }

        double averagePosition = (drive.getLeftMotors().getSelectedSensorPosition() + drive.getRightMotors().getSelectedSensorPosition()) / 2;

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
}