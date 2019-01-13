package org.team3128.prebot.autonomous;

import org.team3128.common.util.enums.Direction;
import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class TestTurn extends CommandGroup {
    public TestTurn(SRXTankDrive drive) {
        addSequential(drive.new CmdInPlaceTurn(90, 10000, Direction.RIGHT));
    }
}