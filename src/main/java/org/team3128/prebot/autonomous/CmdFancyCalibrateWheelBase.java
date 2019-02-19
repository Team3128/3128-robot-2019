package org.team3128.prebot.autonomous;

import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Length;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.drive.SRXTankDrive.CmdCalculateWheelbase;
import org.team3128.common.util.Log;
import com.kauailabs.navx.frc.AHRS;


import edu.wpi.first.wpilibj.command.CommandGroup;


public class CmdFancyCalibrateWheelBase extends CommandGroup {
    public CmdFancyCalibrateWheelBase(AHRS ahrs) {    
        // addSequential(SRXTankDrive.getInstance().new CmdCalculateWheelbase(1.0,0.8,ahrs,200));
        // addSequential(SRXTankDrive.getInstance().new CmdCalculateWheelbase(1.0,0.6,ahrs,300));
        // addSequential(SRXTankDrive.getInstance().new CmdCalculateWheelbase(1.0,0.4,ahrs,400));
        // addSequential(SRXTankDrive.getInstance().new CmdCalculateWheelbase(1.0,0.2,ahrs,500));  
        // addSequential(SRXTankDrive.getInstance().new CmdCalculateWheelbase(0.8,1.0,ahrs,200));
        // addSequential(SRXTankDrive.getInstance().new CmdCalculateWheelbase(0.6,1.0,ahrs,300));
        // addSequential(SRXTankDrive.getInstance().new CmdCalculateWheelbase(0.4,1.0,ahrs,400));
        // addSequential(SRXTankDrive.getInstance().new CmdCalculateWheelbase(0.2,1.0,ahrs,500));
    }
}