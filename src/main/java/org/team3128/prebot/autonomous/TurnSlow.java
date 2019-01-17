package org.team3128.prebot.autonomous;

import org.team3128.common.util.enums.Direction;

import com.esotericsoftware.minlog.Log;

import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class TurnSlow extends CommandGroup {
    public TurnSlow(SRXTankDrive drive) {
        addSequential(drive.new CmdMoveForward(100, 10000, .1));
    }
}