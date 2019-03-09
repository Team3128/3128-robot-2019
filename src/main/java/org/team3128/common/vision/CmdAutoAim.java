package org.team3128.common.vision;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.navigation.Gyro;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.datatypes.PIDConstants;
import org.team3128.common.util.units.Angle;
import org.team3128.gromit.main.MainGromit.GameElement;
import org.team3128.gromit.mechanisms.OptimusPrime;
import org.team3128.gromit.mechanisms.OptimusPrime.RobotState;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;

public class CmdAutoAim extends Command {
    SRXTankDrive drive;
    
    Gyro gyro;

    PIDConstants offsetPID;
    Limelight limelight;

    private final double FEED_FORWARD_POWER = 0.55;
    private final double MINIMUM_POWER = 0.6;
    private final double MINIMUM_POWER_BLIND = 0.3;

    private final double VELOCITY_THRESHOLD = 100;
    private final int VELOCITY_PLATEAU_COUNT = 10;

    double targetHorizontalOffset = -1;
    double tyDecelerateThreshold, decelerateAngleRange;
    DriveCommandRunning cmdRunning;

    private double multiplier;
    
    private double currentHorizontalOffset;
    private double previousVerticalAngle;
    private double currentError, previousError;
    private double currentTime, previousTime;
    private double feedbackPower;
    private double leftVel, rightVel;

    private OptimusPrime op;

    double leftPower;
    double rightPower;
    double horizOffset;

    int tvCount;
    int plateauReachedCount;
    private enum AutoAimState {
        SEARCHING,
        FEEDBACK,
        BLIND,
    }

    private AutoAimState aimState = AutoAimState.SEARCHING;

    public CmdAutoAim(Gyro gyro, Limelight limelight, PIDConstants offsetPID, DriveCommandRunning cmdRunning, double tyDecelerateThreshold, double decelerateAngleRange) {
        this.gyro = gyro;
        this.limelight = limelight;
        this.offsetPID = offsetPID;

        this.cmdRunning = cmdRunning;

        this.tyDecelerateThreshold = tyDecelerateThreshold;
        this.decelerateAngleRange = decelerateAngleRange;
    }

    @Override
    protected void initialize() {
        op = OptimusPrime.getInstance();
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
                } else {
                    tvCount = 0;
                } if(tvCount > 5) {
                    drive.getRightMotors().set(ControlMode.PercentOutput, FEED_FORWARD_POWER);
                    drive.getLeftMotors().set(ControlMode.PercentOutput, FEED_FORWARD_POWER);

                    currentHorizontalOffset = limelight.getValue("tx", 5);
                    Log.info("CmdDynamicAdjust", String.valueOf(currentHorizontalOffset));

                    previousTime = RobotController.getFPGATime();
                    previousError = targetHorizontalOffset - currentHorizontalOffset;

                    gyro.setAngle(currentHorizontalOffset);

                    cmdRunning.isRunning = true;

                    aimState = AutoAimState.FEEDBACK;
                }

                break;

            case FEEDBACK:
                if (!limelight.hasValidTarget()) {
                    Log.info("CmdAutoAim", "No valid target.");

                    if (previousVerticalAngle > 20 * Angle.DEGREES) {
                        Log.info("CmdAutoAim", "Going to blind.");

                        aimState = AutoAimState.BLIND;
                        //drive.tankDrive(MINIMUM_POWER, MINIMUM_POWER);

                        cmdRunning.isRunning = true;
                    }
                    else {
                        aimState = AutoAimState.SEARCHING;

                        cmdRunning.isRunning = false;
                    }
                }
                else {
                    currentHorizontalOffset = limelight.getValue("tx", 5);
                    //currentHorizontalOffset = gyro.getAngle();

                    currentTime = RobotController.getFPGATime() / 1000000.0;
                    currentError = targetHorizontalOffset - currentHorizontalOffset;

                    /**
                     * PID feedback loop for the left and right powers based on the horizontal offset errors.
                     */
                    feedbackPower = 0;

                    feedbackPower += offsetPID.kP * currentError;
                    feedbackPower += offsetPID.kD * (currentError - previousError)/(currentTime - previousTime);
                    
                    rightPower = RobotMath.clamp(FEED_FORWARD_POWER - feedbackPower, -1, 1);
                    leftPower = RobotMath.clamp(FEED_FORWARD_POWER + feedbackPower, -1, 1);
                    
                    Log.info("CmdDynamicAdjust", "L: " + leftPower + "; R: " + rightPower);
                    
                    previousVerticalAngle = limelight.getValue("ty", 1);
                    Log.info("CmdAutoAim", "DECELERATING INIT");
                    multiplier = 1.0 - (1.0 - MINIMUM_POWER) * RobotMath.clamp(1.5*(previousVerticalAngle - tyDecelerateThreshold)/(decelerateAngleRange - tyDecelerateThreshold), 0.0, 1.0);
                    Log.info("CmdAutoAim", "multipier = " + multiplier);

                    drive.tankDrive(-multiplier * rightPower, multiplier * leftPower);

                    previousTime = currentTime;
                    previousError = currentError;
                    Log.info("CmdAutoAim", "Error:" + currentError);
                }
                
                
                break;

            case BLIND:
                drive.stopMovement();
                Log.info("CmdAutoAim", "Driving blind...");
                break;
        }
    }

    @Override
    protected boolean isFinished() {
        if(aimState == AutoAimState.BLIND || aimState == AutoAimState.FEEDBACK) {
            leftVel = Math.abs(drive.getLeftMotors().getSelectedSensorVelocity(0));
            rightVel = Math.abs(drive.getRightMotors().getSelectedSensorVelocity(0));

            if (leftVel < VELOCITY_THRESHOLD && rightVel < VELOCITY_THRESHOLD){
                plateauReachedCount++;
            }
            else {
                plateauReachedCount = 0;
            }

            if(plateauReachedCount >= VELOCITY_PLATEAU_COUNT){
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