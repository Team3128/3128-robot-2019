package org.team3128.prebot.autonomous;

import org.team3128.common.util.enums.Direction;

import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CmdInPlaceTurnTest extends CommandGroup {
    public CmdInPlaceTurnTest(SRXTankDrive drive) {
        addSequential(drive.new CmdInPlaceTurn(360, 1.0, 10000, Direction.RIGHT));
    }
}