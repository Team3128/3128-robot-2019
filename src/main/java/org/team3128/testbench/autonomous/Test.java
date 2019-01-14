package org.team3128.testbench.autonomous;

import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Length;
import org.team3128.common.drive.SRXTankDrive;


import edu.wpi.first.wpilibj.command.CommandGroup;


public class Test extends CommandGroup {
    public Test(SRXTankDrive drive) {

        super();
        final double arc_distance = 2 * Length.ft;

        final float angle = 45f;
        final double large_turn_radius = arc_distance * 180 / (angle * Math.PI);
        //final float small_turn_radius = (float) (arc_distance * 160 / (angle * Math.PI));
        final float small_forward = (float) (1 * Length.ft);
        //final double small_arc_distance = (3 * Length.ft);

        addSequential(drive.new CmdMoveForward(small_forward, 6, true));

        addSequential(drive.new CmdFancyArcTurn(large_turn_radius, angle, 9, Direction.LEFT, 1.0));

        addSequential(drive.new CmdFancyArcTurn(large_turn_radius, angle, -9, Direction.LEFT, 1.0));

        addSequential(drive.new CmdMoveForward(small_forward, 10, true));


    }
}