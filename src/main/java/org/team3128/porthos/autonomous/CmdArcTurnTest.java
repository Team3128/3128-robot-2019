package org.team3128.porthos.autonomous;

import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Length;


import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;


public class CmdArcTurnTest extends CommandGroup {
    public CmdArcTurnTest() {
        SRXTankDrive drive = SRXTankDrive.getInstance();

        addSequential(drive.new CmdArcTurn(36 * Length.in, 90, Direction.LEFT, .75, 10000));
    }

}