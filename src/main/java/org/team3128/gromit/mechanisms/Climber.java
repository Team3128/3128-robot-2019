package org.team3128.gromit.mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team3128.common.autonomous.primitives.CmdRunInParallel;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.util.Log;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class Climber {
    Piston climbPistons;
    TalonSRX backLegMotor;
    
    private static Climber instance = null;
	public static Climber getInstance() {
		if (instance != null) {
			return instance;
		}

		Log.fatal("Climber", "Attempted to get instance before initializtion! Call initialize(...) first.");
		return null;
    }
    
    public static void initialize(Piston climbPistons, TalonSRX backLegMotor) {
        instance = new Climber(climbPistons, backLegMotor);
    }

    private Climber(Piston climbPistons, TalonSRX backLegMotor) {
        this.climbPistons = climbPistons;
        this.backLegMotor = backLegMotor;
    }

    public class CmdClimb1to2 extends CommandGroup {
        SRXTankDrive drive;

        public CmdClimb1to2() {
            drive = SRXTankDrive.getInstance();
            drive.shiftToLow();

            addSequential(new CmdDeployClimberPiston());
            addSequential(drive.new CmdDriveUntilStop(0.6, 1500));
            addSequential(new CmdRetractClimberPiston());
            
            addSequential(new CmdSetBackLegPosition(10000, 3000));

            addSequential(drive.new CmdDriveUntilStop(0.3, 700));

            //addSequential(new CmdSetBackLegPosition(50, 2000));
            addSequential(new CmdRunInParallel(
                drive.new CmdDriveUntilStop(0.5, 2000),
                new CmdSetBackLegPosition(50, 2000)
            ));
        }
    }

    public class CmdClimb2to3 extends CommandGroup {
        SRXTankDrive drive;

        public CmdClimb2to3() {
            drive = SRXTankDrive.getInstance();
            drive.shiftToLow();

            addSequential(new CmdDeployClimberPiston());
            addSequential(drive.new CmdDriveUntilStop(0.6, 1000));
            addSequential(new CmdRetractClimberPiston());

            addSequential(new CmdSetBackLegPosition(18500, 5000));

            addSequential(drive.new CmdDriveUntilStop(0.3, 3000));

            addSequential(new CmdSetBackLegPosition(0, 2000));
        }
    }

    public class CmdDeployClimberPiston extends Command {
        public CmdDeployClimberPiston() {
            super(2.5);
        }

        @Override
        protected void initialize() {
            climbPistons.setPistonOn();
        }

        @Override
        protected boolean isFinished() {
            return isTimedOut();
        }
    }

    public class CmdRetractClimberPiston extends Command {
        public CmdRetractClimberPiston() {
            super(1.5);
        }

        @Override
        protected void initialize() {
            climbPistons.setPistonOff();
        }

        @Override
        protected boolean isFinished() {
            return isTimedOut();
        }
    }

    public class CmdSetBackLegPosition extends Command {
        boolean extending;
        double desiredPosition, currentPosition;
        
        public CmdSetBackLegPosition(int desiredPosition, int timeoutMs) {
            super(timeoutMs / 1000.0);

            this.desiredPosition = desiredPosition;
        }

        @Override
        protected void initialize() {
            extending = desiredPosition > backLegMotor.getSelectedSensorPosition();
            backLegMotor.set(ControlMode.PercentOutput, (extending) ? 1.0 : -1.0);
        }

        @Override
        protected boolean isFinished() {
            currentPosition = backLegMotor.getSelectedSensorPosition();

            if ( extending && currentPosition > desiredPosition) return true;
            if (!extending && currentPosition < desiredPosition) return true;

            return isTimedOut();
        }

        @Override
        protected void end() {
            backLegMotor.set(ControlMode.PercentOutput, 0);

            if (isTimedOut()) {
                Log.unusual("CmdSetBackLegPosition", "Timed out.");
            }
        }

        @Override
        protected void interrupted() {
            end();
        }
    }
}