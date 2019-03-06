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
import org.team3128.gromit.main.MainGromit;
import org.team3128.gromit.main.MainGromit.GameElement;
import org.team3128.gromit.main.MainGromit.ScoreTarget;
import org.team3128.gromit.mechanisms.OptimusPrime.RobotState;
import org.team3128.gromit.util.DeepSpaceConstants;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team3128.common.hardware.navigation.Gyro;


public class CmdAutoPrime extends CommandGroup {
    public CmdAutoPrime(Gyro gyro, Limelight limelight, DriveCommandRunning cmdRunning, PIDConstants offsetPID) {
        SRXTankDrive drive = SRXTankDrive.getInstance();

        addSequential(//new CmdRunInParallel(
            new CmdAutoAim(gyro, limelight, offsetPID, cmdRunning, DeepSpaceConstants.LOWER_TY_DECELERATE_THRESHOLD, 20.0 * Angle.DEGREES)//,
            //new CmdAutOptimusPrime(limelight)
        );
    }
}