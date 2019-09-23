package org.team3128.athos.autonomous.deprecated;

import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.hardware.gyroscope.Gyro;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.utility.enums.Direction;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.units.Length;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CmdPleaseWorkTurnTest extends CommandGroup {
    public CmdPleaseWorkTurnTest(Gyro gyro) {
        PIDConstants innerPID = new PIDConstants(0, 0.1, 0, 0);
        PIDConstants outerPID = new PIDConstants(0, 0.1, 0, 0);

        addSequential(SRXTankDrive.getInstance().new CmdPleaseWorkTurn(gyro, 90 * Angle.DEGREES, 4 * Length.ft,
                Direction.LEFT, 0.7, innerPID, outerPID, 6000));
    }
}