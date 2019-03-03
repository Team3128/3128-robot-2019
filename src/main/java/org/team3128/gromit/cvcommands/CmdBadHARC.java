package org.team3128.gromit.cvcommands;

import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Length;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.limelight.LimelightData;
import org.team3128.common.util.RobotMath;
import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.team3128.common.util.Log;


public class CmdBadHARC extends CommandGroup {
    SRXTankDrive drive;

    private class NumberSet {
        public double radius, desiredYaw, angleTurnInPlace, angleArc;
    }

    Direction inPlaceDir, arcDir;
    NumberSet ns;

    public CmdBadHARC(Limelight limelight) {
        SRXTankDrive drive = SRXTankDrive.getInstance();
        ns = new NumberSet();

        //addSequential(new CmdLambda(() -> {
            LimelightData data = limelight.getValues(20);

            double x = data.x() - limelight.centerDist * RobotMath.tan(data.yaw());
            double z = -1 * data.z() + limelight.centerDist;

            ns.radius = (Math.pow(x, 2) + Math.pow(z, 2))/(2*Math.abs(x));

            ns.desiredYaw = RobotMath.atan(z / (ns.radius - Math.abs(x)));
            ns.angleTurnInPlace = ns.desiredYaw - data.yaw();

            ns.angleArc = ns.desiredYaw;
            
            if (ns.angleTurnInPlace < 0) {
                inPlaceDir = Direction.RIGHT;
                ns.angleTurnInPlace *= -1;
            } 
            else {
                inPlaceDir = Direction.LEFT;
            }

            if (ns.angleArc > 0) {
                arcDir = Direction.RIGHT;
            }
            else {
                arcDir = Direction.LEFT;
                ns.angleArc *= -1;
            }
            
            Log.info("CmdSimpleCVServo", "R: " + ns.radius);
            Log.info("CmdSimpleCVServo", "In-Place Angle: " + ns.angleTurnInPlace);
            Log.info("CmdSimpleCVServo", "Arc Angle: " + ns.angleArc);
        //}));
        
        addSequential(drive.new CmdInPlaceTurn(ns.angleTurnInPlace, inPlaceDir, 1.0, 10000));
        addSequential(drive.new CmdArcTurn(ns.radius * Length.in, ns.angleArc, arcDir, .75, 10000));
    }
}