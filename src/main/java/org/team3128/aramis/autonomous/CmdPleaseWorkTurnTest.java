package org.team3128.aramis.autonomous;

import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.hardware.navigation.Gyro;
import org.team3128.common.util.datatypes.PIDConstants;
import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Angle;
import org.team3128.common.util.units.Length;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CmdPleaseWorkTurnTest extends CommandGroup {
    public CmdPleaseWorkTurnTest(Gyro gyro) {
        PIDConstants innerPID = new PIDConstants(0, 0.1, 0, 0);
        PIDConstants outerPID = new PIDConstants(0, 0.1, 0, 0);

        addSequential(SRXTankDrive.getInstance().new CmdPleaseWorkTurn(gyro, 90 * Angle.DEGREES, 4 * Length.ft, Direction.LEFT, 0.7, innerPID, outerPID, 6000));
    }
}