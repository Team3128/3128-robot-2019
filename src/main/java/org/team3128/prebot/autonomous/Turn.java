package org.team3128.prebot.autonomous;

import org.team3128.common.util.enums.Direction;

import com.esotericsoftware.minlog.Log;

import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class Turn extends CommandGroup {
    public Turn(SRXTankDrive drive) {
        addSequential(drive.new CmdInPlaceTurn(90, .75, 10000, Direction.RIGHT));
    }
}