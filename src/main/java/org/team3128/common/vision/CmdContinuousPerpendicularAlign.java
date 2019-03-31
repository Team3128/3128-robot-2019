package org.team3128.common.vision;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.gromit.util.DeepSpaceConstants;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.hardware.limelight.LEDMode;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.limelight.LimelightKey;
import org.team3128.common.hardware.navigation.Gyro;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.datatypes.PIDConstants;
import org.team3128.common.util.units.Angle;
import org.team3128.common.util.units.Length;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;

public class CmdContinuousPerpendicularAlign extends Command {
    SRXTankDrive drive;
    Gyro gyro;

    Limelight limelight;

    DriveCommandRunning cmdRunning;

    private final double VELOCITY_THRESHOLD = 100;
    private final int VELOCITY_PLATEAU_COUNT = 10;


    private boolean visionStating;

    // MTA Variables
    private double x, y, yaw;

    // MTB Variables
    PIDConstants mtAPID, mtBPID;

    double decelerationStartDistance, decelerationEndDistance;
    double targetHeight, goalHorizontalOffset;

    private double currentHorizontalOffset;
    private double previousVerticalAngle, approximateDistance;

    private double currentAngle;

    private double currentError, previousError;
    private double currentTime, previousTime;

    private double feedbackPower, multiplier;
    private double leftVel, rightVel;

    private double leftPower, rightPower;

    int tvCount;
    int plateauReachedCount;

    private enum ContinuousPerpendicularAlignState {
        SEARCHING, MTB_TURNING, MTB_APPROACHING, MTA_FEEDBACK;
    }

    private ContinuousPerpendicularAlignState aimState;

    public CmdContinuousPerpendicularAlign(Gyro gyro, Limelight limelight, PIDConstants mtAPID, DriveCommandRunning cmdRunning,
            double goalHorizontalOffset, double targetHeight, double decelerationStartDistance, double decelerationEndDistance,
            PIDConstants mtBPID, boolean visionStating) {
        this.gyro = gyro;
        this.limelight = limelight;
        this.mtAPID = mtAPID;

        this.cmdRunning = cmdRunning;

        this.goalHorizontalOffset = goalHorizontalOffset;

        this.targetHeight = targetHeight;

        this.decelerationStartDistance = decelerationStartDistance;
        this.decelerationEndDistance = decelerationEndDistance;

        this.mtBPID = mtBPID;
        this.visionStating = visionStating;
    }

    @Override
    protected void initialize() {
        drive = SRXTankDrive.getInstance();
        limelight.setLEDMode(LEDMode.ON);

        // Don't think this is a good thing...
        if(limelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5) <= DeepSpaceConstants.VISION_TX_ALIGN_THRESHOLD){
            goalHorizontalOffset = 0;
        } else {
            double tempX = limelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);
            double tempY = limelight.getValue(LimelightKey.VERTICAL_OFFSET, 5);
            goalHorizontalOffset = RobotMath.atan((DeepSpaceConstants.VISION_TARGET_POINT + tempX)/tempY) - RobotMath.atan(tempX/tempY);
        }

        cmdRunning.isRunning = false;

        aimState = ContinuousPerpendicularAlignState.SEARCHING;
    }

    @Override
    protected void execute() {
        switch (aimState) {
            case SEARCHING:                
                if (limelight.hasValidTarget()) {
                    tvCount += 1;
                }
                else {
                    tvCount = 0;
                }
                
                if (tvCount > 5) {
                    drive.tankDrive(mtBPID.kF, mtBPID.kF);

                    currentHorizontalOffset = limelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);
                    Log.info("CmdDynamicAdjust", String.valueOf(currentHorizontalOffset));

                    previousTime = RobotController.getFPGATime();
                    previousError = goalHorizontalOffset - currentHorizontalOffset;

                    cmdRunning.isRunning = true;

                    aimState = ContinuousPerpendicularAlignState.MTA_FEEDBACK;
                }

                break;

            case MTB_TURNING:
                // IDK how to get xyyaw but do that here

                break;

            case MTA_FEEDBACK:
                if (limelight.hasValidTarget()) {
                    if(limelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5) <= DeepSpaceConstants.VISION_TX_ALIGN_THRESHOLD){
                        goalHorizontalOffset = 0;
                    }
                    currentHorizontalOffset = limelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 5);

                    currentTime = RobotController.getFPGATime();
                    currentError = goalHorizontalOffset - currentHorizontalOffset;

                    /**
                     * PID feedback loop for the left and right powers based on the horizontal offset errors.
                     */
                    feedbackPower = 0;

                    feedbackPower += mtAPID.kP * currentError;
                    feedbackPower += mtAPID.kD * (currentError - previousError)/(currentTime - previousTime);
                    
                    leftPower = RobotMath.clamp(mtAPID.kF - feedbackPower, -1, 1);
                    rightPower = RobotMath.clamp(mtAPID.kF + feedbackPower, -1, 1);
                    
                    Log.info("CmdAutoAim", "L: " + leftPower + "; R: " + rightPower);
                    
                    previousVerticalAngle = limelight.getValue(LimelightKey.VERTICAL_OFFSET, 2);
                    approximateDistance = limelight.calculateYPrimeFromTY(previousVerticalAngle, targetHeight);

                    Log.info("CmdAutoAim", "distance = " + (approximateDistance / Length.in) + " inches");

                    multiplier = 1.0 - (1.0 - mtAPID.kF) * RobotMath.clamp((decelerationStartDistance - approximateDistance)/(decelerationStartDistance - decelerationEndDistance), 0.0, 1.0);
                    Log.info("CmdAutoAim", "Power Multipier = " + multiplier);

                    drive.tankDrive(multiplier * leftPower, multiplier * rightPower);

                    previousTime = currentTime;
                    previousError = currentError;
                    Log.info("CmdAutoAim", "Error:" + currentError);
                }
            
                break;
        }
    }

    @Override
    protected boolean isFinished() {
        if (aimState == ContinuousPerpendicularAlignState.MTA_FEEDBACK) {
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
        limelight.setLEDMode(LEDMode.OFF);

        cmdRunning.isRunning = false;

        Log.info("CmdAutoAim", "Command Finished.");
    }

    @Override
    protected void interrupted() {
        drive.stopMovement();
        limelight.setLEDMode(LEDMode.OFF);

        cmdRunning.isRunning = false;

        Log.info("CmdAutoAim", "Command Interrupted.");
    }
}