package org.team3128.common.vision;

import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.navigation.Gyro;
import org.team3128.common.util.datatypes.PIDConstants;
import org.team3128.common.util.units.Angle;
import org.team3128.gromit.util.DeepSpaceConstants;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CmdPerpendicularDriveToTarget extends CommandGroup {
    public CmdPerpendicularDriveToTarget(
            Gyro gyro, Limelight bottomLimelight, Limelight topLimelight, DriveCommandRunning cmdRunning, double targetHeight,
            PIDConstants approachPID, double wallIntersectDistance, double xThreshold,
            PIDConstants visionPID, PIDConstants blindPID) {

        addSequential(new CmdApproachPerpendicular(
            topLimelight, cmdRunning, targetHeight,
            approachPID, wallIntersectDistance, xThreshold));

        addSequential(new CmdHorizontalOffsetFeedbackDrive(
            gyro, bottomLimelight, topLimelight, cmdRunning, targetHeight,
            visionPID, -1 * Angle.DEGREES, DeepSpaceConstants.DECELERATE_START_DISTANCE, DeepSpaceConstants.DECELERATE_END_DISTANCE,
            blindPID, 20 * Angle.DEGREES));
    }
}