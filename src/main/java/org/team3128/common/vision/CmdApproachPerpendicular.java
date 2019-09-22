package org.team3128.common.vision;

import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.hardware.limelight.Compute2D;
import org.team3128.common.hardware.limelight.LEDMode;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.limelight.Compute2D.Compute2DLocalization;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.utility.enums.Direction;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;

public class CmdApproachPerpendicular extends Command {
    private SRXTankDrive drive;

    private Limelight limelight;

    private DriveCommandRunning cmdRunning;

    // SEARCHING Variables
    private int tvCount;

    // DIRECTION_FINDING Variables
    private Direction side;

    // APPROACHING_PERPENDICULAR variables
    private Compute2DLocalization locale;
    PIDConstants approachPID;

    private double targetHeight, goalWallIntersectDistance, xThreshold;

    private double currentWallIntersectDistance;

    private double currentError, previousError;
    private double currentTime, previousTime;

    private double feedbackPower;
    private double leftPower, rightPower;

    private enum ApproachPerpendicularState {
        SEARCHING, DIRECTION_FINDING, APPROACHING_PERPENDICULAR, IN_RANGE;
    }

    private ApproachPerpendicularState aimState;

    public CmdApproachPerpendicular(Limelight limelight, DriveCommandRunning cmdRunning, double targetHeight,
            PIDConstants approachPID, double wallIntersectDistance, double xThreshold) {

        this.limelight = limelight;
        this.cmdRunning = cmdRunning;

        this.approachPID = approachPID;

        this.targetHeight = targetHeight;
        this.goalWallIntersectDistance = wallIntersectDistance;
        this.xThreshold = xThreshold;
    }

    @Override
    protected void initialize() {
        drive = SRXTankDrive.getInstance();
        limelight.setLEDMode(LEDMode.ON);

        cmdRunning.isRunning = false;

        aimState = ApproachPerpendicularState.SEARCHING;
    }

    @Override
    protected void execute() {
        switch (aimState) {
        case SEARCHING:
            if (limelight.hasValidTarget()) {
                tvCount += 1;
            } else {
                tvCount = 0;
            }

            if (tvCount > 5) {
                Log.info("ContinuousPerpendicularAlignState", "Target found.");
                Log.info("ContinuousPerpendicularAlignState", "Switching to DIRECTION_FINDING...");

                cmdRunning.isRunning = true;

                aimState = ApproachPerpendicularState.DIRECTION_FINDING;
            }

            break;

        case DIRECTION_FINDING:
            if (!limelight.hasValidTarget()) {
                Log.info("ContinuousPerpendicularAlignState", "Target lost.");
                Log.info("ContinuousPerpendicularAlignState", "Switching to SEARCHING...");

                aimState = ApproachPerpendicularState.SEARCHING;
            } else {
                locale = Compute2D.compute2D(limelight, Compute2D.getInput(limelight, 2), targetHeight);

                if (locale.x < xThreshold /** or it's not facing the target */
                ) {
                    Log.info("ContinuousPerpendicularAlignState", "Entered x threshold of perpendicular.");
                    Log.info("ContinuousPerpendicularAlignState", "Switching to IN_RANGE...");

                    aimState = ApproachPerpendicularState.IN_RANGE;
                } else {
                    // SOMEHOW FIGURE OUT WHICH DIRECTION WE'RE COMING FROM

                    side = Direction.RIGHT;

                    currentWallIntersectDistance = locale.y * RobotMath.tan(locale.yaw);

                    previousTime = RobotController.getFPGATime();
                    previousError = goalWallIntersectDistance - currentWallIntersectDistance;

                    aimState = ApproachPerpendicularState.APPROACHING_PERPENDICULAR;
                }
            }

            break;

        case APPROACHING_PERPENDICULAR:
            locale = Compute2D.compute2D(limelight, Compute2D.getInput(limelight, 2), targetHeight);
            currentWallIntersectDistance = locale.y * RobotMath.tan(locale.yaw);

            currentTime = RobotController.getFPGATime();
            currentError = goalWallIntersectDistance - currentWallIntersectDistance;

            if (currentError <= 0) {
                /**
                 * PID feedback loop for the left and right powers based on the error in the
                 * wall intersect distance.
                 */
                feedbackPower = 0;

                feedbackPower += approachPID.kP * currentError;
                feedbackPower += approachPID.kD * (currentError - previousError) / (currentTime - previousTime);

                leftPower = RobotMath.clamp(approachPID.kF + (side == Direction.RIGHT ? -1 : 1) * feedbackPower, -1, 1);
                rightPower = RobotMath.clamp(approachPID.kF + (side == Direction.RIGHT ? 1 : -1) * feedbackPower, -1,
                        1);

                drive.tankDrive(leftPower, rightPower);
            }

            if (locale.x < xThreshold) {
                Log.info("ContinuousPerpendicularAlignState", "Entered x threshold of perpendicular.");
                Log.info("ContinuousPerpendicularAlignState", "Switching to IN_RANGE...");

                aimState = ApproachPerpendicularState.IN_RANGE;
            }

            break;

        case IN_RANGE:
            break;
        }
    }

    @Override
    protected boolean isFinished() {
        return aimState == ApproachPerpendicularState.IN_RANGE;
    }

    @Override
    protected void end() {
        Log.info("CmdContinuousPependicularAlign", "Arrived in mtA zone.");
    }

    @Override
    protected void interrupted() {
        drive.stopMovement();
        limelight.setLEDMode(LEDMode.OFF);

        cmdRunning.isRunning = false;

        Log.info("CmdContinuousPependicularAlign", "Command Interrupted.");
    }
}