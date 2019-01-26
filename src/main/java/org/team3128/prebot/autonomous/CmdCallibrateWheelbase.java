package org.team3128.prebot.autonomous;

import java.util.ArrayList;
import java.util.List;

import com.esotericsoftware.minlog.Log;

import org.team3128.common.autonomous.primitives.CmdLambda;
import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CmdCallibrateWheelbase extends CommandGroup {
    public CmdCallibrateWheelbase(double duration, double leftSpeed, double rightSpeed) {
        List<Double> calculatedWheelbases = new ArrayList<Double>();
        int numSamples = 5;

        for (int i = 0; i < numSamples; i++) {
            calculatedWheelbases.add(-1.0);

            addSequential(SRXTankDrive.getInstance().new CmdDetermineWheelbase(duration, leftSpeed, rightSpeed, calculatedWheelbases.get(i)));
        }

        addSequential(new CmdLambda(() -> {
            double averageWheelbase = 0;

            for (double wheelbase : calculatedWheelbases) {
                averageWheelbase += wheelbase;
            }
            averageWheelbase /= numSamples;

            Log.info("AverageWheelBase", "" + averageWheelbase);
        }));
    }
}