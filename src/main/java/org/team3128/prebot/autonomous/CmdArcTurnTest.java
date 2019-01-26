package org.team3128.prebot.autonomous;

import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Length;


import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;


public class CmdArcTurnTest extends CommandGroup {
    public CmdArcTurnTest() {
        SRXTankDrive drive = SRXTankDrive.getInstance();
        
        //addSequential(drive.new CmdMoveDistance(MoveEndMode.EITHER, 100, 100, true, .75, true, 10000));
        addSequential(drive.new CmdArcTurn(-48 * Length.in, 90, Direction.LEFT, .75, 10000));
        //addSequential(drive.new CmdMoveDistance(MoveEndMode.BOTH, -254*Length.cm, -254*Length.cm, true, 1, false, 10000));
        //drive.getLeftMotors().set(ControlMode.PercentOutput, -100);
        //drive.getRightMotors().set(ControlMode.PercentOutput, -100);
    }
}