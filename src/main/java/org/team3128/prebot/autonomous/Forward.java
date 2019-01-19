package org.team3128.prebot.autonomous;

import org.team3128.common.util.enums.Direction;

import com.esotericsoftware.minlog.Log;

import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class Forward extends CommandGroup {
    public Forward(SRXTankDrive drive) {
        addSequential(drive.new CmdMoveForward(1000, 100000, .5));
    }
}