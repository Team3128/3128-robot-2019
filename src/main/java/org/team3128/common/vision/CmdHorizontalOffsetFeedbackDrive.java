package org.team3128.common.vision;

import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.drive.calibrationutility.DriveCalibrationUtility;
import org.team3128.common.hardware.limelight.LEDMode;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.limelight.LimelightKey;
import org.team3128.common.hardware.limelight.StreamMode;
import org.team3128.common.hardware.navigation.Gyro;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.utility.units.Angle;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;

public class CmdHorizontalOffsetFeedbackDrive extends Command {
    SRXTankDrive drive;
    Gyro gyro;

    DriveCalibrationUtility dcu;

    Limelight txLimelight;
    Limelight distanceLimelight;

    // private final double FEED_FORWARD_POWER = 0.55;
    // private final double MINIMUM_POWER = 0.1;

    private final double VELOCITY_THRESHOLD = 100;
    private final int VELOCITY_PLATEAU_COUNT = 10;

    double decelerationStartDistance, decelerationEndDistance;
    DriveCommandRunning cmdRunning;

    private PIDConstants visionPID, blindPID;

    private double multiplier;

    private double goalHorizontalOffset;
    private double targetHeight;

    private double currentHorizontalOffset;
    private double previousVerticalAngle, approximateDistance;

    private double currentAngle;

    private double currentError, previousError;
    private double currentTime, previousTime;

    private double feedbackPower;
    private double leftVel, rightVel;

    private double leftPower, rightPower;

    private double blindThreshold;

    private boolean isLowHatch;

    int targetFoundCount;
    int plateauReachedCount;

    private enum HorizontalOffsetFeedbackDriveState {
        SEARCHING, FEEDBACK, BLIND;
    }

    private HorizontalOffsetFeedbackDriveState aimState = HorizontalOffsetFeedbackDriveState.SEARCHING;

    public CmdHorizontalOffsetFeedbackDrive(Gyro gyro, Limelight txLimelight, Limelight distanceLimelight,
            DriveCommandRunning cmdRunning, double targetHeight, PIDConstants visionPID, double goalHorizontalOffset,
            double decelerationStartDistance, double decelerationEndDistance, PIDConstants blindPID,
            double blindThreshold) {// , boolean isLowHatch) {

        this.gyro = gyro;
        this.txLimelight = txLimelight;
        this.distanceLimelight = distanceLimelight;
        this.visionPID = visionPID;

        this.cmdRunning = cmdRunning;

        this.goalHorizontalOffset = goalHorizontalOffset;

        this.targetHeight = targetHeight;

        this.decelerationStartDistance = decelerationStartDistance;
        this.decelerationEndDistance = decelerationEndDistance;

        this.blindPID = blindPID;
        this.blindThreshold = blindThreshold;

        this.isLowHatch = isLowHatch;
    }

    @Override
    protected void initialize() {
        drive = SRXTankDrive.getInstance();
        dcu = DriveCalibrationUtility.getInstance();

        txLimelight.setLEDMode(LEDMode.ON);
        distanceLimelight.setLEDMode(LEDMode.ON);
        if (isLowHatch) {
            distanceLimelight.setStreamMode(StreamMode.LIMELIGHT_CAMERA);
        }
        cmdRunning.isRunning = false;
    }

    @Override
    protected void execute() {
        switch (aimState) {
        case SEARCHING:
            NarwhalDashboard.put("align_status", "searching");
            if (txLimelight.hasValidTarget() && distanceLimelight.hasValidTarget()) {
                targetFoundCount += 1;
            } else {
                targetFoundCount = 0;
            }

            if (targetFoundCount > 5) {
                Log.info("CmdAutoAim", "Target found.");
                Log.info("CmdAutoAim", "Switching to FEEDBACK...");

                drive.tankDrive(visionPID.kF, visionPID.kF);

                currentHorizontalOffset = txLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                previousTime = RobotController.getFPGATime();
                previousError = goalHorizontalOffset - currentHorizontalOffset;

                cmdRunning.isRunning = true;

                aimState = HorizontalOffsetFeedbackDriveState.FEEDBACK;
            }

            break;

        case FEEDBACK:
            NarwhalDashboard.put("align_status", "feedback");
            if (!txLimelight.hasValidTarget() && !distanceLimelight.hasValidTarget()) {
                Log.info("CmdAutoAim", "No valid target.");
                if ((distanceLimelight.cameraAngle > 0 ? 1 : -1) * previousVerticalAngle > blindThreshold) {
                    Log.info("CmdAutoAim", "Switching to BLIND...");

                    gyro.setAngle(0);
                    aimState = HorizontalOffsetFeedbackDriveState.BLIND;
                } else {
                    Log.info("CmdAutoAim", "Returning to SEARCHING...");

                    aimState = HorizontalOffsetFeedbackDriveState.SEARCHING;

                    cmdRunning.isRunning = false;
                }
            } else {
                currentHorizontalOffset = txLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                currentTime = RobotController.getFPGATime();
                currentError = goalHorizontalOffset - currentHorizontalOffset;

                /**
                 * PID feedback loop for the left and right powers based on the horizontal
                 * offset errors.
                 */
                feedbackPower = 0;

                feedbackPower += visionPID.kP * currentError;
                feedbackPower += visionPID.kD * (currentError - previousError) / (currentTime - previousTime);

                leftPower = RobotMath.clamp(visionPID.kF - feedbackPower, -1, 1);
                rightPower = RobotMath.clamp(visionPID.kF + feedbackPower, -1, 1);

                previousVerticalAngle = distanceLimelight.getValue(LimelightKey.VERTICAL_OFFSET, 2);
                approximateDistance = distanceLimelight.calculateYPrimeFromTY(previousVerticalAngle, targetHeight);

                multiplier = 1.0 - (1.0 - blindPID.kF / visionPID.kF)
                        * RobotMath.clamp((decelerationStartDistance - approximateDistance)
                                / (decelerationStartDistance - decelerationEndDistance), 0.0, 1.0);

                drive.tankDrive(multiplier * leftPower, multiplier * rightPower);

                previousTime = currentTime;
                previousError = currentError;
            }

            break;

        case BLIND:
            NarwhalDashboard.put("align_status", "blind");

            currentAngle = gyro.getAngle();

            currentTime = RobotController.getFPGATime() / 1000000.0;
            currentError = -currentAngle;

            /**
             * PID feedback loop for the left and right powers based on the gyro angle
             */
            feedbackPower = 0;

            feedbackPower += blindPID.kP * currentError;
            feedbackPower += blindPID.kD * (currentError - previousError) / (currentTime - previousTime);

            rightPower = RobotMath.clamp(blindPID.kF - feedbackPower, -1, 1);
            leftPower = RobotMath.clamp(blindPID.kF + feedbackPower, -1, 1);

            Log.info("CmdAutoAim", "L: " + leftPower + "; R: " + rightPower);

            drive.tankDrive(leftPower, rightPower);

            previousTime = currentTime;
            previousError = currentError;
            Log.info("CmdAutoAim", "Error:" + currentError);

            break;
        }
    }

    @Override
    protected boolean isFinished() {
        if (aimState == HorizontalOffsetFeedbackDriveState.BLIND) {
            leftVel = Math.abs(drive.getLeftMotors().getSelectedSensorVelocity(0));
            rightVel = Math.abs(drive.getRightMotors().getSelectedSensorVelocity(0));

            if (leftVel < VELOCITY_THRESHOLD && rightVel < VELOCITY_THRESHOLD) {
                plateauReachedCount += 1;
            } else {
                plateauReachedCount = 0;
            }

            if (plateauReachedCount >= VELOCITY_PLATEAU_COUNT) {
                return true;
            }
        }
        return false;
    }

    @Override
    protected void end() {
        drive.stopMovement();
        if (isLowHatch) {
            distanceLimelight.setStreamMode(StreamMode.DRIVER_CAMERA);
        }
        txLimelight.setLEDMode(LEDMode.OFF);
        distanceLimelight.setLEDMode(LEDMode.OFF);

        NarwhalDashboard.put("align_status", "blind");

        cmdRunning.isRunning = false;

        Log.info("CmdAutoAim", "Command Finished.");
    }

    @Override
    protected void interrupted() {
        drive.stopMovement();
        if (isLowHatch) {
            distanceLimelight.setStreamMode(StreamMode.DRIVER_CAMERA);
        }
        txLimelight.setLEDMode(LEDMode.OFF);
        distanceLimelight.setLEDMode(LEDMode.OFF);

        NarwhalDashboard.put("align_status", "blind");

        cmdRunning.isRunning = false;

        Log.info("CmdAutoAim", "Command Finished.");
    }
}