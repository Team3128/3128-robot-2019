package org.team3128.gromit.mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team3128.common.autonomous.primitives.CmdDelay;
import org.team3128.common.autonomous.primitives.CmdRunInParallel;
import org.team3128.common.autonomous.primitives.CmdRunInSeries;
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

    public class CmdClimb1to2Turbo extends CommandGroup {
        SRXTankDrive drive;

        public CmdClimb1to2Turbo() {
            drive = SRXTankDrive.getInstance();
            drive.shiftToLow();

            addSequential(
                new CmdRunInParallel(
                    new CmdRezeroBackLeg(1000),
                    new CmdRunInSeries(
                        new CmdDeployClimberPiston(1400),
                        drive.new CmdDriveUntilStop(1.0, 1100),
                        new CmdRetractClimberPiston(1000)//300
                    )
                )
            );
            
            addSequential(new CmdRunInParallel(
                new CmdSetBackLegPosition(11500, 3000),

                new CmdRunInSeries(
                    new CmdDelay(0.75),
                    drive.new CmdDriveUntilStop(0.8, 1500)
                )
            ));

            addSequential(new CmdRezeroAndPull(0.6, 2000));
        }
    }

    public class CmdClimb2to3Turbo extends CommandGroup {
        SRXTankDrive drive;

        public CmdClimb2to3Turbo() {
            drive = SRXTankDrive.getInstance();
            drive.shiftToLow();

            addSequential(new CmdDeployClimberPiston(1400));
            addSequential(drive.new CmdDriveUntilStop(1.0, 1200));
            addSequential(new CmdRetractClimberPiston(1000));

            addSequential(new CmdSetBackLegPosition(18000, 3000));

            addSequential(drive.new CmdDriveUntilStop(0.5, 2000));

            addSequential(new CmdRezeroAndPull(0.6, 4000));

        }
    }

    public class CmdClimb1to2 extends CommandGroup {
        SRXTankDrive drive;

        public CmdClimb1to2() {
            drive = SRXTankDrive.getInstance();
            drive.shiftToLow();

            addSequential(new CmdRunInParallel(
                new CmdRezeroBackLeg(3000),

                new CmdRunInSeries(
                    new CmdDeployClimberPiston(1800),
                    drive.new CmdDriveUntilStop(0.5, 1200),
                    new CmdRetractClimberPiston(1400)
                )
            ));
            
            addSequential(new CmdRunInParallel(
                new CmdSetBackLegPosition(11350, 3000),

                new CmdRunInSeries(
                    new CmdDelay(0.75),
                    drive.new CmdDriveUntilStop(0.6, 1200)
                )
            ));

            addSequential(new CmdRezeroAndPull(0.7, 3850));
        }
    }

    public class CmdClimb2to3 extends CommandGroup {
        SRXTankDrive drive;

        public CmdClimb2to3() {
            drive = SRXTankDrive.getInstance();
            drive.shiftToLow();

            addSequential(new CmdDeployClimberPiston(2500));
            addSequential(drive.new CmdDriveUntilStop(0.5, 1500));
            addSequential(new CmdRetractClimberPiston(1500));


            addSequential(new CmdRunInParallel(
                new CmdSetBackLegPosition(18500, 3000),

                new CmdRunInSeries(
                    new CmdDelay(1.25),
                    drive.new CmdDriveUntilStop(0.4, 1500)
                )
            ));

            addSequential(new CmdPartialRezeroAndPull(0.6, 5000));
        }
    }


    public class CmdDeployClimberPiston extends Command {
        public CmdDeployClimberPiston(int timeoutMs) {
            super(timeoutMs / 1000.0);
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
        public CmdRetractClimberPiston(int timeoutMs) {
            super(timeoutMs / 1000.0);
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

    public class CmdRezeroBackLeg extends Command {
        public CmdRezeroBackLeg(int timeoutMs) {
            super(timeoutMs / 1000.0);
        }

        @Override
        protected void initialize() {
            backLegMotor.set(ControlMode.PercentOutput, -1.0);
        }

        @Override
        protected boolean isFinished() {
            if (timeSinceInitialized() < 0.2) return false;
            return isTimedOut() || Math.abs(backLegMotor.getSelectedSensorVelocity()) < 50;
        }

        @Override
        protected void end() {
            backLegMotor.set(ControlMode.PercentOutput, 0);
            backLegMotor.setSelectedSensorPosition(0);
        }

        @Override
        protected void interrupted() {
            end();
        }
    }

    
    public class CmdRezeroAndPull extends Command {
        double power;

        public CmdRezeroAndPull(double power, int timeoutMs) {
            super(timeoutMs / 1000.0);

            this.power = power;
        }

        @Override
        protected void initialize() {
            SRXTankDrive.getInstance().tankDrive(power, power);

            backLegMotor.set(ControlMode.PercentOutput, -1.0);
        }

        @Override
        protected boolean isFinished() {
            if (timeSinceInitialized() < 0.2) return false;
            return isTimedOut() || Math.abs(backLegMotor.getSelectedSensorVelocity()) < 50;
        }

        @Override
        protected void end() {
            backLegMotor.set(ControlMode.PercentOutput, 0);
            backLegMotor.setSelectedSensorPosition(0);

            SRXTankDrive.getInstance().stopMovement();
        }

        @Override
        protected void interrupted() {
            end();
        }
    }

    public class CmdPartialRezeroAndPull extends Command {
        double power;

        public CmdPartialRezeroAndPull(double power, int timeoutMs) {
            super(timeoutMs / 1000.0);

            this.power = power;
        }

        @Override
        protected void initialize() {
            SRXTankDrive.getInstance().tankDrive(power, power);

            backLegMotor.set(ControlMode.PercentOutput, -1.0);
        }
        
        @Override
        protected boolean isFinished() {
            if (timeSinceInitialized() < 0.2) return false;
            return isTimedOut() || Math.abs(backLegMotor.getSelectedSensorPosition()) < 7000;
        }

        @Override
        protected void end() {
            backLegMotor.set(ControlMode.PercentOutput, 0);
            backLegMotor.setSelectedSensorPosition(0);

            SRXTankDrive.getInstance().stopMovement();
        }

        @Override
        protected void interrupted() {
            end();
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