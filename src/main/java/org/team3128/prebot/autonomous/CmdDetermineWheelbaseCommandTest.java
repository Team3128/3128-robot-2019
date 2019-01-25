package org.team3128.prebot.autonomous;

import com.esotericsoftware.minlog.Log;
import com.kauailabs.navx.frc.AHRS;
import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CmdDetermineWheelbaseCommandTest extends CommandGroup {
    public CmdDetermineWheelbaseCommandTest(AHRS ahrs, SRXTankDrive drive, double duration, double leftSpeed, double rightSpeed) {
        addSequential(drive.new CmdDetermineWheelbase(ahrs, drive, duration, leftSpeed, rightSpeed));
        double averageWheelBase0 = drive.returnWheelBase();
        addSequential(drive.new CmdDetermineWheelbase(ahrs, drive, duration, leftSpeed, rightSpeed));
        double averageWheelBase1 = drive.returnWheelBase();
        addSequential(drive.new CmdDetermineWheelbase(ahrs, drive, duration, leftSpeed, rightSpeed));
        double averageWheelBase2 = drive.returnWheelBase();
        addSequential(drive.new CmdDetermineWheelbase(ahrs, drive, duration, leftSpeed, rightSpeed));
        double averageWheelBase3 = drive.returnWheelBase();
        addSequential(drive.new CmdDetermineWheelbase(ahrs, drive, duration, leftSpeed, rightSpeed));
        double averageWheelBase4 = drive.returnWheelBase();

        double averageWheelBase = (averageWheelBase0 + averageWheelBase1 + averageWheelBase2 + averageWheelBase3 + averageWheelBase4)/5;

        Log.info("AverageWheelBase", "" + averageWheelBase);
    }
}