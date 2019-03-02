package org.team3128.gromit.cvcommands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.drive.TankDrive;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.limelight.LimelightData;
import org.team3128.common.hardware.navigation.Gyro;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.datatypes.PIDConstants;
import org.team3128.common.util.units.Angle;
import org.team3128.common.util.units.Length;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;

public class CmdAutoAim extends Command {
    SRXTankDrive drive;
    
    Gyro gyro;

    PIDConstants offsetPID;
    Limelight limelight;


    private final double TARGET_HORIZONTAL_OFFSET = 0 * Angle.DEGREES;
    private final double FEED_FORWARD_POWER = 0.7;

    private final double VELOCITY_THRESHOLD = 100;
    private final int VELOCITY_PLATEAU_COUNT = 10;
    
    private double currentHorizontalOffset;
    private double previousVerticalAngle;
    private double currentError, previousError;
    private double currentTime, previousTime;
    private double feedbackPower;
    private double leftVel, rightVel;

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

    public CmdAutoAim(Gyro gyro, Limelight limelight, PIDConstants offsetPID) {
        this.gyro = gyro;
        this.limelight = limelight;
        this.offsetPID = offsetPID;
    }

    @Override
    protected void initialize() {
        drive = SRXTankDrive.getInstance();
    }

    @Override
    protected void execute() {
        switch (aimState) {
            case SEARCHING:
                // TODO: Move to FEEDBACK when tv reads 1 <TODO: determine what this number this actually is> times.
                
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
                    previousError = TARGET_HORIZONTAL_OFFSET - currentHorizontalOffset;

                    gyro.setAngle(0);

                    aimState = AutoAimState.FEEDBACK;
                }

                break;

            case FEEDBACK:
                if (limelight.hasValidTarget()) {
                    if (previousVerticalAngle > 20 * Angle.DEGREES) {
                        aimState = AutoAimState.BLIND;
                        drive.tankDrive(0.3, 0.3);
                    } else {
                        aimState = AutoAimState.SEARCHING;
                    }
                    
                }

                currentHorizontalOffset = limelight.getValue("tx", 5);

				currentTime = RobotController.getFPGATime() / 1000000.0;
				currentError = TARGET_HORIZONTAL_OFFSET - currentHorizontalOffset;
				
				/**
				 * PID feedback loop for the left and right powers based on the horizontal offset errors.
				 */
				feedbackPower = 0;

				feedbackPower += offsetPID.kP * currentError;
				feedbackPower += offsetPID.kD * (currentError - previousError)/(currentTime - previousTime);
                
                rightPower = RobotMath.clamp(FEED_FORWARD_POWER - feedbackPower, -1, 1);
                leftPower = RobotMath.clamp(FEED_FORWARD_POWER + feedbackPower, -1, 1);
                
				Log.info("CmdDynamicAdjust", "L: " + leftPower + "; R: " + rightPower);
                
                drive.tankDrive(rightPower, leftPower);

				previousTime = currentTime;
                previousError = currentError;
                previousVerticalAngle = limelight.getValue("ty", 1);
                
                break;

            case BLIND:
                break;
        }
    }

    @Override
    protected boolean isFinished() {
        if(aimState == AutoAimState.BLIND){
            leftVel = Math.abs(drive.getLeftMotors().getSelectedSensorVelocity(0));
            rightVel = Math.abs(drive.getRightMotors().getSelectedSensorVelocity(0));

            if(leftVel < VELOCITY_THRESHOLD && rightVel < VELOCITY_THRESHOLD){
                plateauReachedCount++;
            } else {
                plateauReachedCount = 0;
            }
            if(plateauReachedCount>= VELOCITY_PLATEAU_COUNT){
                return true;
            }
        }
        return false;
    }

    @Override
    protected void end() {
        drive.stopMovement();
    }

    @Override
    protected void interrupted() {
        end();
    }
}