package org.team3128.prebot.autonomous;

import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Length;
import org.team3128.common.drive.SRXTankDrive;


import edu.wpi.first.wpilibj.command.CommandGroup;


public class Test extends CommandGroup {
    public Test(SRXTankDrive drive) {
        
        super();
        final double arc_distance = 8 * Length.ft;

        final float angle = 45f;
        final double large_turn_radius = arc_distance * 180 / (angle * Math.PI);
        //final float small_turn_radius = (float) (arc_distance * 160 / (angle * Math.PI));
        //final float small_forward = (float) (1 * Length.ft);
        //final double small_arc_distance = (3 * Length.ft);

        //addSequential(drive.new CmdMoveForward(small_forward, 10000, false));

        //addSequential(drive.new CmdFancyArcTurn(large_turn_radius, angle, 10000, Direction.LEFT, 0.1));
        int leftPos1 = drive.getLeftMotors().getSelectedSensorPosition();
        int rightPos1 = drive.getRightMotors().getSelectedSensorPosition();
        addSequential(drive.new CmdFancierArcTurn(large_turn_radius, angle, 10000, Direction.LEFT, 1));
        int leftPos2 = drive.getLeftMotors().getSelectedSensorPosition();
        int rightPos2 = drive.getRightMotors().getSelectedSensorPosition();

        //addSequential(drive.new CmdMoveForward(small_forward, 10000, false));

    }
}