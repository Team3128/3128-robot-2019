package org.team3128.prebot.autonomous;

import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Length;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.util.Log;
import com.kauailabs.navx.frc.AHRS;


import edu.wpi.first.wpilibj.command.CommandGroup;


public class CmdFancyCalibrateWheelBase extends CommandGroup {
    public CmdFancyCalibrateWheelBase() {    
        addSequential()    
    }
}