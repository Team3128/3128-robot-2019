package org.team3128.gromit.autonomous;

import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Length;
import org.team3128.common.util.datatypes.PIDConstants;

import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.autonomous.primitives.CmdRunInParallel;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.vision.CmdAutoAim;
import org.team3128.gromit.cvcommands.CmdAutOptimusPrime;
import org.team3128.gromit.main.MainGromit;
import org.team3128.gromit.main.MainGromit.GameElement;
import org.team3128.gromit.main.MainGromit.ScoreTarget;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team3128.common.hardware.navigation.Gyro;


public class CmdLevelOneHatch extends CommandGroup {
    public CmdLevelOneHatch(Gyro gyro, Limelight limelight) {
        SRXTankDrive drive = SRXTankDrive.getInstance();
        PIDConstants offsetPID = new PIDConstants(0, 0.0005, 0, 0.00009);
        addSequential(new CmdRunInParallel(
            drive.new CmdTargetAlignSimple(gyro, limelight, 0.3, offsetPID, 10000),
            new CmdAutOptimusPrime(GameElement.HATCH_PANEL, ScoreTarget.CARGO_SHIP)
        ));
    }
}