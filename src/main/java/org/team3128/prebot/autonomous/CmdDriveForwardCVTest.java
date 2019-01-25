package org.team3128.prebot.autonomous;

import org.team3128.common.util.Log;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CmdDriveForwardCVTest extends CommandGroup {

    public NetworkTable table;

    public CmdDriveForwardCVTest() {
        SRXTankDrive drive = SRXTankDrive.getInstance();

        double valCurrent2 = 0.0;

        table = NetworkTableInstance.getDefault().getTable("limelight");

        for(int i = 0; i<2000; i++){
            //Log.info("trigger", "trigger triggered");
        valCurrent2 = table.getEntry("ty").getDouble(0.0);
        }
        valCurrent2 = valCurrent2/2000;

        double d = (28.5 - 5) / (Math.tan((28.0 + valCurrent2) * (Math.PI/180)));
        Log.info("distance calc", String.valueOf(d));
        addSequential(drive.new CmdDriveStraight(-d, 1.0, 10000));

        Log.info("auto_tyav", String.valueOf(valCurrent2));
        //NarwhalDashboard.put("tyav", String.valueOf(valCurrent2));
        valCurrent2 = 0.0;
    }
}