package org.team3128.prebot.autonomous;

import org.team3128.common.util.units.Length;

import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.drive.routemaker.Waypoint;

import edu.wpi.first.wpilibj.command.CommandGroup;


public class CmdRoutemakerTest extends CommandGroup {
    public CmdRoutemakerTest() {
        SRXTankDrive drive = SRXTankDrive.getInstance();

        addSequential(drive.new CmdStaticRouteDrive(0.5, 10000, 
            new Waypoint(0 * Length.ft, 0 * Length.ft, 90, 5 * Length.ft),
            new Waypoint(3 * Length.ft, 4 * Length.ft, 90, 5 * Length.ft)
        ));
    }
}