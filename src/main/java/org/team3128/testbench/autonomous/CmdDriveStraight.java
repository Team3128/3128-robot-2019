package org.team3128.testbench.autonomous;

import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CmdDriveStraight extends CommandGroup {
    public CmdDriveStraight() {
        addSequential(SRXTankDrive.getInstance().new CmdDriveStraight(100, .5, 5000));
    }
}