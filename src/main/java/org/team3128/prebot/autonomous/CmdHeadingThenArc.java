package org.team3128.prebot.autonomous;

import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Length;
import org.team3128.common.hardware.limelight;


import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;


public class CmdHeadingThenArc extends CommandGroup {
    public CmdArcTurnTest(Limelight limelight) {
        SRXTankDrive drive = SRXTankDrive.getInstance();
        LimelightData data = limelight.getValues(20);
        Direction dir;
        double radius = (Math.pow(data.x(),2) + Math.pow(data.y(), 2))/(2*data.x());
        double angleRobotToHorizontal = RobotMath.arctan(data.y()/data.x());
        double angleRadiusToHorizontal = RobotMath.arccos((radius - data.x())/radius);
        double angleTargetToTangent = angleRobotToHorizontal - (90 - angleRadiusToHorizontal);
        

        addSequential(drive.new CmdInPlaceTurn(angleTargetToTangent, Direction.Right, 0.75, 10000));
        addSequential(drive.new CmdArcTurn(radius * Length.in, angleRadiusToHorizontal, Direction.Right, .75, 10000));
    }
}