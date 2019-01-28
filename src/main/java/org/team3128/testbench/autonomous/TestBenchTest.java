package org.team3128.testbench.autonomous;

import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class TestBenchTest extends CommandGroup {
    public TestBenchTest(SRXTankDrive drive) {
        addSequential(drive.new CmdDriveStraight(100, .5, 5000));
    }
}