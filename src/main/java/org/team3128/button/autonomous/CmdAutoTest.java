package org.team3128.button.autonomous;

import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Length;


import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CmdAutoTest extends CommandGroup {
    public CmdAutoTest(){
    SRXTankDrive drive = SRXTankDrive.getInstance();
    
    addSequential(drive.new CmdDriveStraight(100 * Length.in, .5, 10000));
    addSequential(drive.new CmdInPlaceTurn(90, Direction.LEFT, .5, 10000));
    }
}