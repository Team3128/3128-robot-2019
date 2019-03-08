// package org.team3128.gromit.autonomous;

// import org.team3128.gromit.cvcommands.Cmd
// import org.team3128.common.util.enums.Direction;
// import org.team3128.common.util.units.Length;


// import org.team3128.common.drive.SRXTankDrive;

// import edu.wpi.first.wpilibj.command.CommandGroup;


// public class CmdTestDriveTrain extends CommandGroup {
//     public CmdTestDriveTrain() {
//         SRXTankDrive drive = SRXTankDrive.getInstance();

//         addSequential(new CmdAutoAim(gyro, limelight, offsetPID, cmdRunning, DeepSpaceConstants.LOWER_TY_DECELERATE_THRESHOLD, 20.0 * Angle.DEGREES));
//     }
// }