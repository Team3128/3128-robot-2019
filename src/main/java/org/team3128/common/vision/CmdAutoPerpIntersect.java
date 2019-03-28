package org.team3128.common.vision;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.gromit.util.DeepSpaceConstants;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.navigation.Gyro;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.datatypes.PIDConstants;
import org.team3128.common.util.units.Angle;
import org.team3128.common.util.units.Length;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;

public class CmdAutoPerpIntersect extends Command {
    SRXTankDrive drive;
    Gyro gyro;

    Limelight limelight;

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

    private boolean visionStating;

    int tvCount;
    int plateauReachedCount;

    private enum AutoAimState {
        SEARCHING, FEEDBACK, BLIND;
    }

    private AutoAimState aimState = AutoAimState.SEARCHING;

    public CmdAutoPerpIntersect(Gyro gyro, Limelight limelight, PIDConstants visionPID, DriveCommandRunning cmdRunning,
            double goalHorizontalOffset, double targetHeight, double decelerationStartDistance, double decelerationEndDistance,
            PIDConstants blindPID, boolean visionStating) {
        this.gyro = gyro;
        this.limelight = limelight;
        this.visionPID = visionPID;

        this.cmdRunning = cmdRunning;

        this.goalHorizontalOffset = goalHorizontalOffset;

        this.targetHeight = targetHeight;

        this.decelerationStartDistance = decelerationStartDistance;
        this.decelerationEndDistance = decelerationEndDistance;

        this.blindPID = blindPID;
        this.visionStating = visionStating;
    }

    @Override
    protected void initialize() {
        if(limelight.getValue("x", 5) <= DeepSpaceConstants.VISION_TX_ALIGN_THRESHOLD){
            goalHorizontalOffset = 0;
        } else {
            double tempX = limelight.getValue("x", 5);
            double tempY = limelight.getValue("y", 5);
            goalHorizontalOffset = RobotMath.atan((DeepSpaceConstants.VISION_TARGET_POINT + tempX)/tempY) - RobotMath.atan(tempX/tempY);
        }
        drive = SRXTankDrive.getInstance();

        limelight.turnOnLED();
        cmdRunning.isRunning = false;
    }

    @Override
    protected void execute() {
        switch (aimState) {
            case SEARCHING:                
                if (limelight.getValues(1).tv() == 1) {
                    tvCount++;
                }
                else {
                    tvCount = 0;
                }
                
                if (tvCount > 5) {
                    drive.getRightMotors().set(ControlMode.PercentOutput, visionPID.kF);
                    drive.getLeftMotors().set(ControlMode.PercentOutput, visionPID.kF);

                    currentHorizontalOffset = limelight.getValue("tx", 5);
                    Log.info("CmdDynamicAdjust", String.valueOf(currentHorizontalOffset));

                    previousTime = RobotController.getFPGATime();
                    previousError = goalHorizontalOffset - currentHorizontalOffset;

                    cmdRunning.isRunning = true;

                    aimState = AutoAimState.FEEDBACK;
                }

                break;

            case FEEDBACK:
                if (!limelight.hasValidTarget()) {
                    Log.info("CmdAutoAim", "No valid target.");

                    if (previousVerticalAngle > 20 * Angle.DEGREES || (visionStating && previousVerticalAngle > -7 * Angle.DEGREES)) {
                        Log.info("CmdAutoAim", "Going to blind.");

                        gyro.setAngle(0);
                        aimState = AutoAimState.BLIND;
                    }
                    else {
                        Log.info("CmdAutoAim", "Returning to searching...");

                        aimState = AutoAimState.SEARCHING;

                        cmdRunning.isRunning = false;
                    }
                }
                else {
                    if(limelight.getValue("x", 5) <= DeepSpaceConstants.VISION_TX_ALIGN_THRESHOLD){
                        goalHorizontalOffset = 0;
                    }
                    currentHorizontalOffset = limelight.getValue("tx", 5);

                    currentTime = RobotController.getFPGATime();
                    currentError = goalHorizontalOffset - currentHorizontalOffset;

                    /**
                     * PID feedback loop for the left and right powers based on the horizontal offset errors.
                     */
                    feedbackPower = 0;

                    feedbackPower += visionPID.kP * currentError;
                    feedbackPower += visionPID.kD * (currentError - previousError)/(currentTime - previousTime);
                    
                    leftPower = RobotMath.clamp(visionPID.kF - feedbackPower, -1, 1);
                    rightPower = RobotMath.clamp(visionPID.kF + feedbackPower, -1, 1);
                    
                    Log.info("CmdAutoAim", "L: " + leftPower + "; R: " + rightPower);
                    
                    previousVerticalAngle = limelight.getValue("ty", 2);
                    approximateDistance = limelight.calculateDistanceFromTY(previousVerticalAngle, targetHeight);

                    Log.info("CmdAutoAim", "distance = " + (approximateDistance / Length.in) + " inches");

                    multiplier = 1.0 - (1.0 - visionPID.kF) * RobotMath.clamp((decelerationStartDistance - approximateDistance)/(decelerationStartDistance - decelerationEndDistance), 0.0, 1.0);
                    Log.info("CmdAutoAim", "Power Multipier = " + multiplier);

                    drive.tankDrive(multiplier * leftPower, multiplier * rightPower);

                    previousTime = currentTime;
                    previousError = currentError;
                    Log.info("CmdAutoAim", "Error:" + currentError);
                }
                

                break;

            case BLIND:
                currentAngle = gyro.getAngle();

                currentTime = RobotController.getFPGATime() / 1000000.0;
                currentError = -currentAngle;

                /**
                 * PID feedback loop for the left and right powers based on the gyro angle
                 */
                feedbackPower = 0;

                feedbackPower += blindPID.kP * currentError;
                feedbackPower += blindPID.kD * (currentError - previousError)/(currentTime - previousTime);
                
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
        if (aimState == AutoAimState.BLIND) {
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
        limelight.turnOffLED();

        cmdRunning.isRunning = false;

        Log.info("CmdAutoAim", "Command Finished.");
    }

    @Override
    protected void interrupted() {
        drive.stopMovement();
        limelight.turnOffLED();

        cmdRunning.isRunning = false;

        Log.info("CmdAutoAim", "Command Interrupted.");
    }
}