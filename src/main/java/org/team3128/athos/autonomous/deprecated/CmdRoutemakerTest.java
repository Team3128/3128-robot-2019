package org.team3128.athos.autonomous.deprecated;

import org.team3128.common.utility.units.Length;

import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.deprecated.routemaker_deprecated.Waypoint;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CmdRoutemakerTest extends CommandGroup {
    public CmdRoutemakerTest() {
        SRXTankDrive drive = SRXTankDrive.getInstance();

        // addSequential(
        // drive.new CmdStaticRouteDrive(0.5, 10000, new Waypoint(0 * Length.ft, 0 *
        // Length.ft, 90, 5 * Length.ft),
        // new Waypoint(8 * Length.ft, 15 * Length.ft, 90, 5 * Length.ft)));
    }
}