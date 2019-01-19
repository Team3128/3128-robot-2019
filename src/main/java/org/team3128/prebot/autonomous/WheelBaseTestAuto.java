package org.team3128.prebot.autonomous;

import org.team3128.common.util.enums.Direction;
import com.esotericsoftware.minlog.Log;
import com.kauailabs.navx.frc.AHRS;
import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class WheelBaseTestAuto extends CommandGroup {
    public WheelBaseTestAuto(AHRS ahrs, SRXTankDrive drive, double duration, double leftSpeed, double rightSpeed) {
        addSequential(drive.new WheelBaseTest(ahrs, drive, duration, leftSpeed, rightSpeed));
    }
}