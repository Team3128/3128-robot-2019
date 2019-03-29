package org.team3128.common.vision;

import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.navigation.Gyro;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.datatypes.PIDConstants;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;

public class CmdContinuousPursuit extends Command {
    SRXTankDrive drive;
    Gyro gyro;

    Limelight limelight;

    private final double VELOCITY_THRESHOLD = 100;
    private final int VELOCITY_PLATEAU_COUNT = 10;

    private double decelerationStartDistance, decelerationStopDistance;
    private double finalPower;
    DriveCommandRunning cmdRunning;

    private PIDConstants visionPID;

    private double multiplier;

    private double goalHorizontalOffset, goalSkew;
    private double currentHorizontalOffset, currentSkew;

    private double targetHeight;
    private double previousVerticalAngle, approximateDistance;

    private double currentAngleError, previousAngleError;
    private double skewError;
    private double currentTime, previousTime;

    private double feedbackPower;
    private double leftFeedforwardMultiplier, rightFeedforwardMultiplier;
    private double leftVel, rightVel;

    private double leftPower, rightPower;

    private int targetFoundCount;
    private int plateauReachedCount;

    private enum ContinuousPursuitState {
        SEARCHING, FEEDBACK;
    }

    private ContinuousPursuitState aimState = ContinuousPursuitState.SEARCHING;

    public CmdContinuousPursuit(Gyro gyro, Limelight limelight, PIDConstants visionPID, DriveCommandRunning cmdRunning,
            double goalHorizontalOffset, double targetHeight, double decelerationStartDistance, double decelerationStopDistance,
            double finalPower) {
        this.gyro = gyro;
        this.limelight = limelight;
        this.visionPID = visionPID;

        this.cmdRunning = cmdRunning;

        this.goalHorizontalOffset = goalHorizontalOffset;

        this.targetHeight = targetHeight;

        this.decelerationStartDistance = decelerationStartDistance;
        this.decelerationStopDistance = decelerationStopDistance;

        this.finalPower = finalPower;
    }

    @Override
    protected void initialize() {
        drive = SRXTankDrive.getInstance();

        limelight.turnOnLED();
        cmdRunning.isRunning = false;
    }

    @Override
    protected void execute() {
        switch (aimState) {
            case SEARCHING:                
                if (limelight.hasValidTarget()) {
                    targetFoundCount += 1;
                }
                else {
                    targetFoundCount = 0;
                }
                
                if (targetFoundCount > 5) {
                    Log.info("CmdAutoAim", "Target found.");
                    Log.info("CmdAutoAim", "Switching to FEEDBACK...");

                    drive.tankDrive(visionPID.kF, visionPID.kF);

                    currentHorizontalOffset = limelight.getValue("tx", 5);

                    previousTime = RobotController.getFPGATime();
                    previousAngleError = goalHorizontalOffset - currentHorizontalOffset;

                    cmdRunning.isRunning = true;

                    aimState = ContinuousPursuitState.FEEDBACK;
                }

                break;

            case FEEDBACK:
                if (!limelight.hasValidTarget()) {
                    Log.info("CmdAutoAim", "No valid target.");
                    Log.info("CmdAutoAim", "Returning to SEARCHING...");

                    aimState = ContinuousPursuitState.SEARCHING;

                    cmdRunning.isRunning = false;
                }
                else {
                    currentHorizontalOffset = limelight.getValue("tx", 5);
                    currentSkew = limelight.getValue("ts", 5);

                    currentTime = RobotController.getFPGATime();

                    /**
                     * PID feedback loop for the left and right powers based on the horizontal offset errors.
                     * Postitive angle error means the target is to the right of the robot.
                     */
                    currentAngleError = goalHorizontalOffset - currentHorizontalOffset;

                    feedbackPower = 0;

                    feedbackPower += visionPID.kP * currentAngleError;
                    feedbackPower += visionPID.kD * (currentAngleError - previousAngleError)/(currentTime - previousTime);

                    /**
                     * Adjusting the motor feed-forward proportionally to current skew
                     */

                    skewError = goalSkew - currentSkew;

                    leftFeedforwardMultiplier = (currentAngleError > 0)

                    
                    /**
                     * Set the motor powers.
                     */
                    leftPower = RobotMath.clamp(visionPID.kF - feedbackPower, -1, 1);
                    rightPower = RobotMath.clamp(visionPID.kF + feedbackPower, -1, 1);
                    
                    
                    previousVerticalAngle = limelight.getValue("ty", 2);
                    approximateDistance = limelight.calculateDistanceFromTY(previousVerticalAngle, targetHeight);


                    multiplier = 1.0 - (1.0 - finalPower / visionPID.kF) * RobotMath.clamp((decelerationStartDistance - approximateDistance)/(decelerationStartDistance - decelerationStopDistance), 0.0, 1.0);

                    drive.tankDrive(multiplier * leftPower, multiplier * rightPower);

                    previousTime = currentTime;
                    previousAngleError = currentAngleError;
                }
                
                break;
        }
    }

    @Override
    protected boolean isFinished() {
        if (aimState == ContinuousPursuitState.FEEDBACK) {
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