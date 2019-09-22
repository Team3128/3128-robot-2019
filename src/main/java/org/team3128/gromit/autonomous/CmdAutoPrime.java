package org.team3128.gromit.autonomous;

import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.vision.CmdHorizontalOffsetFeedbackDrive;
import org.team3128.gromit.cvcommands.CmdAutOptimusPrime;
import org.team3128.gromit.main.MainDeepSpaceRobot.GameElement;
import org.team3128.gromit.main.MainDeepSpaceRobot.ScoreTarget;
import org.team3128.gromit.constants.GameConstants;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team3128.common.hardware.navigation.Gyro;

public class CmdAutoPrime extends CommandGroup {

    public CmdAutoPrime(Gyro gyro, Limelight bottomLimelight, Limelight topLimelight, DriveCommandRunning cmdRunning,
            PIDConstants visionPID, PIDConstants blindPID, GameElement gameElement, ScoreTarget scoreTarget,
            boolean intakingHatchPanel) {
        double targetHeight = GameConstants.getVisionTargetHeight(gameElement, scoreTarget);

        Limelight distLimelight = bottomLimelight;
        Limelight txLimelight = topLimelight;

        if (gameElement == GameElement.HATCH_PANEL
                && (scoreTarget == ScoreTarget.ROCKET_LOW || scoreTarget == ScoreTarget.CARGO_SHIP)) {
            distLimelight = topLimelight;
            // isLowHatch = true;
        }

        if (scoreTarget == ScoreTarget.ROCKET_MID || scoreTarget == ScoreTarget.ROCKET_TOP
                || (scoreTarget == ScoreTarget.ROCKET_LOW && gameElement == GameElement.CARGO)
                || (scoreTarget == ScoreTarget.CARGO_SHIP && gameElement == GameElement.CARGO)) {
            txLimelight = bottomLimelight;
            // useBottom = true;
        }

        addSequential(new CmdAutOptimusPrime(gameElement, scoreTarget, intakingHatchPanel, 500));

        addSequential(// new CmdRunInParallel(
                new CmdHorizontalOffsetFeedbackDrive(gyro, txLimelight, distLimelight, cmdRunning, targetHeight,
                        visionPID, -2 * Angle.DEGREES, GameConstants.DECELERATE_START_DISTANCE,
                        GameConstants.DECELERATE_END_DISTANCE, blindPID, 20 * Angle.DEGREES)// ,
        // new CmdStreamUpdate(bottomLimelight, topLimelight, useBottom)
        );
    }
}