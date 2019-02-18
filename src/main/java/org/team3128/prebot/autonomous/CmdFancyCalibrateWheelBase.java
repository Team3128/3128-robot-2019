package org.team3128.prebot.autonomous;

import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Length;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.drive.SRXTankDrive.CmdCalculateWheelbase;
import org.team3128.common.util.Log;
import com.kauailabs.navx.frc.AHRS;


import edu.wpi.first.wpilibj.command.CommandGroup;


public class CmdFancyCalibrateWheelBase extends CommandGroup {
    public CmdFancyCalibrateWheelBase() {    
        addSequential(SRXTankDrive.getInstance().new CmdCalculateWheelbase(1.0,0.8,3));
        addSequential(SRXTankDrive.getInstance().new CmdCalculateWheelbase(1.0,0.6,3));
        addSequential(SRXTankDrive.getInstance().new CmdCalculateWheelbase(1.0,0.4,3));
        addSequential(SRXTankDrive.getInstance().new CmdCalculateWheelbase(1.0,0.2,3));  
        addSequential(SRXTankDrive.getInstance().new CmdCalculateWheelbase(0.8,1.0,3));
        addSequential(SRXTankDrive.getInstance().new CmdCalculateWheelbase(0.6,1.0,3));
        addSequential(SRXTankDrive.getInstance().new CmdCalculateWheelbase(0.4,1.0,3));
        addSequential(SRXTankDrive.getInstance().new CmdCalculateWheelbase(0.2,1.0,3));
    }
}