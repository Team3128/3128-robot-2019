package org.team3128.prebot.autonomous;

import org.team3128.common.util.enums.Direction;
import com.esotericsoftware.minlog.Log;
import com.kauailabs.navx.frc.AHRS;
import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class WheelBaseTestAuto extends CommandGroup {
    public WheelBaseTestAuto(SRXTankDrive drive, AHRS ahrs) {
        addSequential(drive.new WheelBaseTest(ahrs, drive, 2000, 2000, 4000));
    }
}