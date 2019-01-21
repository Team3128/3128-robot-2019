package org.team3128.testbench.autonomous;

import com.esotericsoftware.minlog.Log;

import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class TestBenchTest extends CommandGroup {
    public TestBenchTest(SRXTankDrive drive) {
        addSequential(drive.new CmdMoveForward(100, 5000, .5));
    }
}