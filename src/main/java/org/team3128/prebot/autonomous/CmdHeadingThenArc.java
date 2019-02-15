package org.team3128.prebot.autonomous;

import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Length;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.limelight.LimelightData;
import org.team3128.common.util.RobotMath;

import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;


public class CmdHeadingThenArc extends CommandGroup {
    public CmdHeadingThenArc(Limelight limelight) {
        SRXTankDrive drive = SRXTankDrive.getInstance();
        LimelightData data = limelight.getValues(20);
        Direction dir = Direction.RIGHT;
        // double radius = (Math.pow(data.x(), 2) + Math.pow(data.y(), 2))/(2*data.x());
        double radius = (Math.pow(3, 2) + Math.pow(10, 2))/(2*3);
        // double angleRobotToHorizontal = RobotMath.atan(data.y()/data.x());
        double angleRobotToHorizontal = RobotMath.atan(10/3);
        // double angleRadiusToHorizontal = RobotMath.acos((radius - data.x())/radius);
        double angleRadiusToHorizontal = RobotMath.acos((radius - 3)/radius);

        double angleTargetToTangent = angleRobotToHorizontal - (90 - angleRadiusToHorizontal);
        

        addSequential(drive.new CmdInPlaceTurn(angleTargetToTangent, dir, 0.75, 10000));
        addSequential(drive.new CmdArcTurn(radius * Length.in, (int) angleRadiusToHorizontal, dir, .75, 10000));
    }
}