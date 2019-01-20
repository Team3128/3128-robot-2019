package org.team3128.prebot.autonomous;

import org.team3128.common.util.units.Length;
import org.team3128.common.util.Log;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import org.team3128.common.util.enums.Direction;

import org.team3128.common.util.Log;

import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ForwardCV extends CommandGroup {

    public NetworkTable table;

    public ForwardCV(SRXTankDrive drive) {

        double valCurrent2 = 0.0;

        table = NetworkTableInstance.getDefault().getTable("limelight");

        for(int i = 0; i<2000; i++){
            //Log.info("trigger", "trigger triggered");
            valCurrent2 = valCurrent2 + table.getEntry("ty").getDouble(0.0);
        }
        valCurrent2 = valCurrent2/2000;

        double d = (25 - 5) / Math.tan(28.0 + valCurrent2);

        addSequential(drive.new CmdMoveForward((d * Length.in), 10000, true));

        Log.info("auto_tyav", String.valueOf(valCurrent2));
        //NarwhalDashboard.put("tyav", String.valueOf(valCurrent2));
        valCurrent2 = 0.0;
    }
}