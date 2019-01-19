package org.team3128.prebot.autonomous;

import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Length;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.esotericsoftware.minlog.Log;

import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.drive.SRXTankDrive.MoveEndMode;

import edu.wpi.first.wpilibj.command.CommandGroup;


public class Forward extends CommandGroup {
    public Forward(SRXTankDrive drive) {
        //addSequential(drive.new CmdMoveDistance(MoveEndMode.EITHER, 100, 100, true, .75, true, 10000));
        addSequential(drive.new CmdMoveForward(-254, 10000, 1));
        //addSequential(drive.new CmdMoveDistance(MoveEndMode.BOTH, 70000*Length.cm, 700000*Length.cm, true, 1, false, 10000));
        //drive.getLeftMotors().set(ControlMode.PercentOutput, -100);
        //drive.getRightMotors().set(ControlMode.PercentOutput, -100);
    }
}