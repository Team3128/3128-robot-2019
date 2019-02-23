package org.team3128.common.drive.calibrationutility;

import org.team3128.common.util.units.Length;

import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;


public class Cmd100InchDrive extends CommandGroup {
    public Cmd100InchDrive() {
        SRXTankDrive drive = SRXTankDrive.getInstance();

        addSequential(drive.new CmdDriveStraight(100 * Length.in, .5, 10000));
    }
}