package org.team3128.prebot.autonomous;

import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Length;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.util.Log;
import com.kauailabs.navx.frc.AHRS;


import edu.wpi.first.wpilibj.command.CommandGroup;


public class CmdWheelbaseTest extends CommandGroup {
    public CmdWheelbaseTest(AHRS ahrs) {        
        SRXTankDrive drive = SRXTankDrive.getInstance();

        final double arc_distance = 4 * Length.ft;

        float angle = 45f;
        final double large_turn_radius = arc_distance * 180 / (angle * Math.PI);
        //final float small_turn_radius = (float) (arc_distance * 160 / (angle * Math.PI));
        //final float small_forward = (float) (1 * Length.ft);
        //final double small_arc_distance = (3 * Length.ft);

        //addSequential(drive.new CmdMoveForward(small_forward, 10000, false));

        //addSequential(drive.new CmdFancyArcTurn(large_turn_radius, angle, 10000, Direction.LEFT, 0.1));
        for(int i = 0; i < 3; i++){
            if(i == 0){
                angle = 90f;
            } else if(i==1){
                angle = 45f;
            } else if(i==2){
                angle = 30f;
            }
            double R;
            double Rleft;
            double Rright;
            double s;
            double wheelbase1;
            double wheelbase2;
            int leftPos = drive.getLeftMotors().getSelectedSensorPosition();
            int rightPos = drive.getRightMotors().getSelectedSensorPosition();
            double theta = Double.valueOf(ahrs.getYaw());
            addSequential(drive.new CmdArcTurn(large_turn_radius, angle, Direction.LEFT, 1, 10000));
            theta = ahrs.getYaw() - theta;
            theta = theta*(Math.PI/180);
            leftPos = drive.getLeftMotors().getSelectedSensorPosition() - leftPos;
            rightPos = drive.getRightMotors().getSelectedSensorPosition() - rightPos;

            s = 0.5*(Double.valueOf(leftPos + rightPos));
        
            R = s / theta;
            Rleft = leftPos / theta;
            Rright = rightPos / theta;


            wheelbase1 = 2*(R - Rright);
            wheelbase2 = 2*(Rleft - R);
            Log.info("round: ", Integer.toString(i));
            Log.info("wb1", Double.toString(wheelbase1));
            Log.info("wb2", Double.toString(wheelbase1));
        }
        //addSequential(drive.new CmdMoveForward(small_forward, 10000, false));

    }
}