package org.team3128.common.vision;

import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.navigation.Gyro;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.utility.units.Angle;
import org.team3128.gromit.constants.GameConstants;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CmdPerpendicularDriveToTarget extends CommandGroup {
        public CmdPerpendicularDriveToTarget(Gyro gyro, Limelight bottomLimelight, Limelight topLimelight,
                        DriveCommandRunning cmdRunning, double targetHeight, PIDConstants approachPID,
                        double wallIntersectDistance, double xThreshold, PIDConstants visionPID,
                        PIDConstants blindPID) {

                addSequential(new CmdApproachPerpendicular(topLimelight, cmdRunning, targetHeight, approachPID,
                                wallIntersectDistance, xThreshold));

                addSequential(new CmdHorizontalOffsetFeedbackDrive(gyro, bottomLimelight, topLimelight, cmdRunning,
                                targetHeight, visionPID, -1 * Angle.DEGREES, GameConstants.DECELERATE_START_DISTANCE,
                                GameConstants.DECELERATE_END_DISTANCE, blindPID, 20 * Angle.DEGREES));// , false));
        }
}