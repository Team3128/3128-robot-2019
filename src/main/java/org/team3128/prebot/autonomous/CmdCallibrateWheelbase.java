package org.team3128.prebot.autonomous;

import java.util.ArrayList;
import java.util.List;

import com.esotericsoftware.minlog.Log;

import org.team3128.common.autonomous.primitives.CmdDelay;
import org.team3128.common.autonomous.primitives.CmdLambda;
import org.team3128.common.autonomous.primitives.CmdLog;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.util.Wheelbase;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CmdCallibrateWheelbase extends CommandGroup {
    public CmdCallibrateWheelbase(double duration, double leftSpeed, double rightSpeed) {
        List<Wheelbase> calculatedWheelbases = new ArrayList<Wheelbase>();
        int numSamples = 5;

        for (int i = 0; i < numSamples; i++) {
            Wheelbase wb = new Wheelbase();

            addSequential(SRXTankDrive.getInstance().new CmdDetermineWheelbase(duration, leftSpeed, rightSpeed, wb));
            addSequential(new CmdDelay(0.5));
            addSequential(new CmdLog("" + wb.wheelbase));

            calculatedWheelbases.add(wb);

        }

        addSequential(new CmdLambda(() -> {
            double averageWheelbase = 0;

            for (Wheelbase wheelbase : calculatedWheelbases) {
                averageWheelbase += wheelbase.wheelbase;
            }
            averageWheelbase /= numSamples;

            Log.info("AverageWheelBase", "" + averageWheelbase);
        }));
    }
}