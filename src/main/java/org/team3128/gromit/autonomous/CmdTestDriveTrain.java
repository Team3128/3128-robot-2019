package org.team3128.gromit.autonomous;

import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Length;


import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;


public class CmdTestDriveTrain extends CommandGroup {
    public CmdTestDriveTrain() {
        SRXTankDrive drive = SRXTankDrive.getInstance();

        addSequential(drive.new CmdDriveStraight(100, .6, 5000));
        addSequential(drive.new CmdInPlaceTurn(90, Direction.RIGHT, .4, 5000));
        addSequential(drive.new CmdArcTurn(50, 45, Direction.LEFT, .4, 5000));
    }
}