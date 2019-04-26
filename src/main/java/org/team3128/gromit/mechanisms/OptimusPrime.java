package org.team3128.gromit.mechanisms;

import org.team3128.gromit.mechanisms.Lift.LiftHeightTarget;
import org.team3128.gromit.mechanisms.LiftIntake.LiftIntakeState;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

import org.team3128.common.autonomous.primitives.CmdRunInParallel;
import org.team3128.common.util.Log;
import org.team3128.common.util.units.Length;
import org.team3128.gromit.main.MainDeepSpaceRobot.GameElement;
import org.team3128.gromit.main.MainDeepSpaceRobot.ScoreTarget;
import org.team3128.gromit.mechanisms.LiftIntake;

import org.team3128.gromit.mechanisms.FourBar.FourBarAngleTarget;
//import org.team3128.gromit.mechanisms.GroundIntake.GroundIntakeState;

/**
 * Overall mechanism wrapper to control the {@link Lift} and {@link LiftIntake}.
 * 
 * @author Chris, Jude, Tygan
 * 
 */

public class OptimusPrime {
    public enum RobotState {
        REST(LiftHeightTarget.BASE, FourBarAngleTarget.VERTICAL),

        INTAKE_FLOOR_CARGO(LiftHeightTarget.INTAKE_FLOOR_CARGO, FourBarAngleTarget.CARGO_INTAKE),

        VISION_STATE(LiftHeightTarget.VISION, FourBarAngleTarget.HATCH_LOW),

        INTAKE_HATCH(LiftHeightTarget.HATCH_INTAKE, FourBarAngleTarget.HATCH_LOW),

        DEPOSIT_SHIP_HATCH(LiftHeightTarget.LOW_HATCH, FourBarAngleTarget.HATCH_LOW),

        DEPOSIT_LOW_HATCH(LiftHeightTarget.LOW_HATCH, FourBarAngleTarget.HATCH_LOW),
        DEPOSIT_MID_HATCH(LiftHeightTarget.MID_HATCH, FourBarAngleTarget.HATCH_HIGH),
        DEPOSIT_TOP_HATCH(LiftHeightTarget.TOP_HATCH, FourBarAngleTarget.HATCH_HIGH),

        DEPOSIT_SHIP_CARGO(LiftHeightTarget.SHIP_CARGO, FourBarAngleTarget.CARGO_SHIP),

        DEPOSIT_LOW_CARGO(LiftHeightTarget.LOW_CARGO, FourBarAngleTarget.CARGO_LOW),
        DEPOSIT_MID_CARGO(LiftHeightTarget.MID_CARGO, FourBarAngleTarget.CARGO_LOW),
        DEPOSIT_TOP_CARGO(LiftHeightTarget.TOP_CARGO, FourBarAngleTarget.CARGO_HIGH);

        public LiftHeightTarget liftHeightTarget;
        public FourBarAngleTarget fourBarAngleTarget;

        private RobotState(LiftHeightTarget liftHeightTarget, FourBarAngleTarget fourBarAngleTarget) {
            this.liftHeightTarget = liftHeightTarget;
            this.fourBarAngleTarget = fourBarAngleTarget;
        }

        public static RobotState getOptimusState(GameElement gameElement, ScoreTarget scoreLevel, boolean intakingHatchPanel) {
            if (intakingHatchPanel) {
                return INTAKE_HATCH;
            }

            if (gameElement == GameElement.CARGO) {
                switch (scoreLevel) {
                case CARGO_SHIP:
                    return DEPOSIT_SHIP_CARGO;
                case ROCKET_LOW:
                    return DEPOSIT_LOW_CARGO;
                case ROCKET_MID:
                    return DEPOSIT_MID_CARGO;
                case ROCKET_TOP:
                    return DEPOSIT_TOP_CARGO;
                }
            } else {
                switch (scoreLevel) {
                case CARGO_SHIP:
                    return DEPOSIT_SHIP_HATCH;
                case ROCKET_LOW:
                    return DEPOSIT_LOW_HATCH;
                case ROCKET_MID:
                    return DEPOSIT_MID_HATCH;
                case ROCKET_TOP:
                    return DEPOSIT_TOP_HATCH;
                }
            }

            return REST;
        }
    }

    Lift lift;
    LiftIntake liftIntake;
    FourBar fourBar;

    public RobotState robotState;
    // GroundIntake groundIntake;

    private static OptimusPrime instance = null;

    public static OptimusPrime getInstance() {
        if (instance != null) {
            return instance;
        }

        Log.fatal("OptimusPrime", "Attempted to get instance before initializtion! Call initialize(...) first.");
        return null;
    }

    public static void initialize() {
        instance = new OptimusPrime();
    }

    private OptimusPrime() {
        lift = Lift.getInstance();
        liftIntake = LiftIntake.getInstance();
        fourBar = FourBar.getInstance();
        // groundIntake = GroundIntake.getInstance();
    }

    public class CmdCascadedOptimus extends Command {
        private RobotState state;
        private int delayMS;

        public CmdCascadedOptimus(RobotState state, int delayMS) {
            this.state = state;
            this.delayMS = delayMS;
        }

        @Override
        protected void initialize() {
            lift.heightControl(state.liftHeightTarget);
        }

        @Override
        protected boolean isFinished() {
            if (timeSinceInitialized() < delayMS / 1000.0) {
                return false;
            }
            else {
                fourBar.setTarget(state.fourBarAngleTarget);
                return true;
            }
        }
    }

    public void setState(RobotState state) {
        new CmdCascadedOptimus(state, 
            (lift.getCurrentHeight() < 10 * Length.in && state == RobotState.DEPOSIT_LOW_HATCH)
             ? 250 : 0).start();

        this.robotState = state;
    }
    
    public class CmdEnterIntakeMode extends CommandGroup {
        public CmdEnterIntakeMode() {      
            addSequential(new CmdRunInParallel(
                lift.new CmdHeightControl(LiftHeightTarget.INTAKE_FLOOR_CARGO),
                liftIntake.new CmdSetLiftIntakeState(LiftIntakeState.CARGO_INTAKE))
            );
            // addSequential(groundIntake.new CmdSetGroundIntakeState(GroundIntakeState.DEPLOYED));
            addSequential(fourBar.new CmdAngleControl(FourBarAngleTarget.CARGO_INTAKE));
            // addSequential(groundIntake.new CmdSetGroundIntakeState(GroundIntakeState.INTAKING));
        }
    }
    
    public class CmdExitIntakeMode extends CommandGroup {
        public CmdExitIntakeMode() {
            // addSequential(groundIntake.new CmdSetGroundIntakeState(GroundIntakeState.DEPLOYED));
            addSequential(new CmdRunInParallel(
                lift.new CmdHeightControl(LiftHeightTarget.BASE),
                liftIntake.new CmdSetLiftIntakeState(LiftIntakeState.DEMOGORGON_HOLDING))
            );
            // addSequential(groundIntake.new CmdSetGroundIntakeState(GroundIntakeState.RETRACTED));
        }
    }
    
    public class CmdDepositGameElement extends CommandGroup {
        public CmdDepositGameElement() {
            //TODO
        }
    }

    /*
    public class CmdSetState extends Command{
        RobotState state;
        public CmdSetState(RobotState robotState) {
            this.state = robotState;
		}
		
		@Override
		protected void initialize() {
				
		}
		
		@Override
		protected void execute() {
        }
		
		@Override
		protected boolean isFinished() {
            if(state == RobotState.STARTING){
                Lift.getInstance().setState(LiftHeightState.STARTING);
                try{
                    Thread.sleep(800);
                } catch(InterruptedException io){
                    io.printStackTrace();
                }
            } else {
                OptimusPrime.getInstance().setState(state);
            }
            return true;
		}
		
		@Override
		protected void end() {
		}
	
    }
    */
}