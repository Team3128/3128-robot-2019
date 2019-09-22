package org.team3128.gromit.cvcommands;

import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.datatypes.PIDConstants;

import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.limelight.StreamMode;
import org.team3128.common.autonomous.primitives.CmdLambda;
import org.team3128.common.autonomous.primitives.CmdRunInParallel;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.vision.CmdHorizontalOffsetFeedbackDrive;
import org.team3128.gromit.cvcommands.CmdAutOptimusPrime;
import org.team3128.gromit.main.MainDeepSpaceRobot.GameElement;
import org.team3128.gromit.main.MainDeepSpaceRobot.ScoreTarget;
import org.team3128.gromit.utility.DeepSpaceConstants;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team3128.common.hardware.navigation.Gyro;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;

public class CmdStreamUpdate extends Command {
    Limelight bottomLimelight;
    Limelight topLimelight;

    boolean useBottom;

    public CmdStreamUpdate(Limelight bottomLimelight, Limelight topLimelight, boolean useBottom) {
        this.bottomLimelight = bottomLimelight;
        this.topLimelight = topLimelight;

        this.useBottom = useBottom;
    }

    @Override
    protected void initialize() {
        topLimelight.setStreamMode(StreamMode.LIMELIGHT_CAMERA);
        NarwhalDashboard.put("streamLL", (useBottom ? "bottom" : "top"));
    }

    @Override
    protected void interrupted() {
        end();
    }

    @Override
    protected void end() {
        topLimelight.setStreamMode(StreamMode.DRIVER_CAMERA);
        NarwhalDashboard.put("streamLL", "top");
        Log.info("CmdStreamUpdate", "Switching back to Streaming Driver Camera");
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}