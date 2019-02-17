package org.team3128.common.drive.callibrationutility;

import com.kauailabs.navx.frc.AHRS;

import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.util.Log;
import org.team3128.common.util.Wheelbase;

public class DriveCallibrationUtility {
    private static DriveCallibrationUtility instance = null;
	public static DriveCallibrationUtility getInstance() {
		if (instance != null) {
			return instance;
		}

		Log.fatal("DriveCallibrationUtility", "Attempted to get instance before initializtion! Call initialize(...) first.");
		return null;
    }
    
    public static void initialize(AHRS ahrs) {
        instance = new DriveCallibrationUtility(ahrs);
    }

    public double maxLeftSpeed = 0;
    public double maxRightSpeed = 0;
	
    public Wheelbase calculatedWheelbase;
    
    public SRXTankDrive drive;
    public AHRS ahrs;

    private DriveCallibrationUtility(AHRS ahrs) {
        maxLeftSpeed = 0;
        maxRightSpeed = 0;

        calculatedWheelbase = new Wheelbase();

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

        calculatedWheelbase = new Wheelbase();
        NarwhalDashboard.addButton("wheelbase", (boolean down) -> {
            if (down) {
                (new CmdCallibrateWheelbase(ahrs, 5000, 1000, 1500, calculatedWheelbase)).start();;
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

        NarwhalDashboard.put("leftKf", getLeftKf());
        NarwhalDashboard.put("rightKf", getRightKf());

        NarwhalDashboard.put("leftSpeedScalar", getLeftSpeedScalar());
        NarwhalDashboard.put("rightSpeedScalar", getRightSpeedScalar());

        NarwhalDashboard.put("wheelBase", calculatedWheelbase.wheelbase);
        NarwhalDashboard.put("leftVelocityError", calculatedWheelbase.leftVelocityError);
        NarwhalDashboard.put("rightVelocityError", calculatedWheelbase.rightVelocityError);
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