package org.team3128.gromit.autonomous;

import org.team3128.common.util.units.Angle;
import org.team3128.common.util.datatypes.PIDConstants;

import org.team3128.common.hardware.limelight.Limelight;

import org.team3128.common.autonomous.primitives.CmdRunInParallel;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.vision.CmdHorizontalOffsetFeedbackDrive;
import org.team3128.gromit.cvcommands.CmdAutOptimusPrime;
import org.team3128.gromit.main.MainDeepSpaceRobot.GameElement;
import org.team3128.gromit.main.MainDeepSpaceRobot.ScoreTarget;
import org.team3128.gromit.util.DeepSpaceConstants;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team3128.common.hardware.navigation.Gyro;


public class CmdAutoPrime extends CommandGroup {

    public CmdAutoPrime(Gyro gyro, Limelight limelight, DriveCommandRunning cmdRunning, PIDConstants visionPID, PIDConstants blindPID, GameElement gameElement, ScoreTarget scoreTarget, boolean intakingHatchPanel) {
        double targetHeight = DeepSpaceConstants.getVisionTargetHeight(gameElement, scoreTarget);
        boolean visionStating = !intakingHatchPanel && gameElement == GameElement.HATCH_PANEL && (scoreTarget == ScoreTarget.CARGO_SHIP || scoreTarget == ScoreTarget.ROCKET_LOW);
        
        addSequential(new CmdRunInParallel(
            new CmdHorizontalOffsetFeedbackDrive(
                gyro, limelight, cmdRunning, targetHeight,
                visionPID, -1 * Angle.DEGREES, DeepSpaceConstants.DECELERATE_START_DISTANCE, DeepSpaceConstants.DECELERATE_END_DISTANCE,
                blindPID, 20 * Angle.DEGREES),
            new CmdAutOptimusPrime(limelight, gameElement, scoreTarget, intakingHatchPanel)
        ));
    }
}