package org.team3128.gromit.autonomous;

import org.team3128.common.util.Log;
import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Length;


import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;


public class CmdAutoTest extends CommandGroup {
    public CmdAutoTest() {
        SRXTankDrive drive = SRXTankDrive.getInstance();
        Log.info("asdfa", "Asdfasd");
        addSequential(drive.new CmdArcTurn(36 * Length.in, 90, Direction.LEFT, .75, 10000));
    }

}