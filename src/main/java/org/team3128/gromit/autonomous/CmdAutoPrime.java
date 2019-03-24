package org.team3128.gromit.autonomous;

import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Angle;
import org.team3128.common.util.units.Length;
import org.team3128.common.util.datatypes.PIDConstants;

import org.team3128.common.hardware.limelight.Limelight;

import org.team3128.common.autonomous.primitives.CmdRunInParallel;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.vision.CmdAutoAim;
import org.team3128.gromit.cvcommands.CmdAutOptimusPrime;
import org.team3128.gromit.main.MainDeepSpaceRobot;
import org.team3128.gromit.main.MainDeepSpaceRobot.GameElement;
import org.team3128.gromit.main.MainDeepSpaceRobot.ScoreTarget;
import org.team3128.gromit.mechanisms.OptimusPrime;
import org.team3128.gromit.mechanisms.OptimusPrime.RobotState;
import org.team3128.gromit.util.DeepSpaceConstants;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team3128.common.hardware.navigation.Gyro;


public class CmdAutoPrime extends CommandGroup {

    public CmdAutoPrime(Gyro gyro, Limelight limelight, DriveCommandRunning cmdRunning, PIDConstants visionPID, PIDConstants blindPID, GameElement gameElement, ScoreTarget scoreTarget, boolean intakingHatchPanel) {
        double targetHeight = DeepSpaceConstants.getVisionTargetHeight(gameElement, scoreTarget);
        boolean visionStating = !intakingHatchPanel && gameElement == GameElement.HATCH_PANEL && (scoreTarget == ScoreTarget.CARGO_SHIP || scoreTarget == ScoreTarget.ROCKET_LOW);
        
        addSequential(new CmdRunInParallel(
            new CmdAutoAim(gyro, limelight, visionPID, cmdRunning,
                -1 * Angle.DEGREES, targetHeight, DeepSpaceConstants.DECELERATE_START_DISTANCE, DeepSpaceConstants.DECELERATE_END_DISTANCE,
                blindPID, visionStating),
            new CmdAutOptimusPrime(limelight, gameElement, scoreTarget, intakingHatchPanel)
        ));
    }
}