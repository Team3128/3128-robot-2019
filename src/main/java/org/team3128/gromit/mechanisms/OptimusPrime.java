package org.team3128.gromit.mechanisms;

import org.team3128.gromit.mechanisms.Lift.LiftHeightState;
import org.team3128.gromit.mechanisms.LiftIntake.LiftIntakeState;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

import org.team3128.common.autonomous.primitives.CmdRunInParallel;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.units.Length;
import org.team3128.gromit.main.MainDeepSpaceRobot.GameElement;
import org.team3128.gromit.main.MainDeepSpaceRobot.ScoreTarget;
import org.team3128.gromit.mechanisms.LiftIntake;

import org.team3128.gromit.mechanisms.FourBar.FourBarState;
//import org.team3128.gromit.mechanisms.GroundIntake.GroundIntakeState;

/**
 * Overall mechanism wrapper to control the {@link Lift} and {@link LiftIntake}.
 * 
 * @author Chris, Jude, Tygan
 * 
 */

public class OptimusPrime {
    public enum RobotState {
        ZERO(LiftHeightState.BASE, FourBarState.VERTICAL),

        INTAKE_FLOOR_CARGO(LiftHeightState.INTAKE_FLOOR_CARGO, FourBarState.CARGO_INTAKE),

        VISION_STATE(LiftHeightState.VISION, FourBarState.HATCH_LOW),

        INTAKE_HATCH(LiftHeightState.HATCH_INTAKE, FourBarState.HATCH_LOW),

        DEPOSIT_SHIP_HATCH(LiftHeightState.LOW_HATCH, FourBarState.HATCH_LOW),

        DEPOSIT_LOW_HATCH(LiftHeightState.LOW_HATCH, FourBarState.HATCH_LOW),
        DEPOSIT_MID_HATCH(LiftHeightState.MID_HATCH, FourBarState.HATCH_HIGH),
        DEPOSIT_TOP_HATCH(LiftHeightState.TOP_HATCH, FourBarState.HATCH_HIGH),

        DEPOSIT_SHIP_CARGO(LiftHeightState.SHIP_CARGO, FourBarState.CARGO_SHIP),

        DEPOSIT_LOW_CARGO(LiftHeightState.LOW_CARGO, FourBarState.CARGO_LOW),
        DEPOSIT_MID_CARGO(LiftHeightState.MID_CARGO, FourBarState.CARGO_LOW),
        DEPOSIT_TOP_CARGO(LiftHeightState.TOP_CARGO, FourBarState.CARGO_HIGH);

        public LiftHeightState targetLiftState;
        public FourBarState targetFourBarState;
        // states for lift intake and ground intakes

        private RobotState(LiftHeightState liftState, FourBarState fourBarState) {
            this.targetLiftState = liftState;
            this.targetFourBarState = fourBarState;
        }

        public static RobotState getOptimusState(GameElement gameElement, ScoreTarget scoreLevel,
                boolean intakingHatchPanel) {
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

            return ZERO;
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
            lift.setState(state.targetLiftState);
        }

        @Override
        protected boolean isFinished() {
            if (timeSinceInitialized() < delayMS / 1000.0) {
                return false;
            } else {
                fourBar.setState(state.targetFourBarState);
                return true;
            }
        }
    }

    public void setState(RobotState state) {
        new CmdCascadedOptimus(state,
                (lift.getCurrentHeight() < 10 * Length.in && state == RobotState.DEPOSIT_LOW_HATCH) ? 250 : 0).start();

        this.robotState = state;
    }

    public class CmdEnterIntakeMode extends CommandGroup {
        public CmdEnterIntakeMode() {
            addSequential(new CmdRunInParallel(lift.new CmdHeightControl(LiftHeightState.INTAKE_FLOOR_CARGO),
                    liftIntake.new CmdSetLiftIntakeState(LiftIntakeState.CARGO_INTAKE)));
            // addSequential(groundIntake.new
            // CmdSetGroundIntakeState(GroundIntakeState.DEPLOYED));
            addSequential(fourBar.new CmdAngleControl(FourBarState.CARGO_INTAKE));
            // addSequential(groundIntake.new
            // CmdSetGroundIntakeState(GroundIntakeState.INTAKING));
        }
    }

    public class CmdExitIntakeMode extends CommandGroup {
        public CmdExitIntakeMode() {
            // addSequential(groundIntake.new
            // CmdSetGroundIntakeState(GroundIntakeState.DEPLOYED));
            addSequential(new CmdRunInParallel(lift.new CmdHeightControl(LiftHeightState.BASE),
                    liftIntake.new CmdSetLiftIntakeState(LiftIntakeState.DEMOGORGON_HOLDING)));
            // addSequential(groundIntake.new
            // CmdSetGroundIntakeState(GroundIntakeState.RETRACTED));
        }
    }

    public class CmdDepositGameElement extends CommandGroup {
        public CmdDepositGameElement() {
            // TODO
        }
    }

    /*
     * public class CmdSetState extends Command{ RobotState state; public
     * CmdSetState(RobotState robotState) { this.state = robotState; }
     * 
     * @Override protected void initialize() {
     * 
     * }
     * 
     * @Override protected void execute() { }
     * 
     * @Override protected boolean isFinished() { if(state == RobotState.STARTING){
     * Lift.getInstance().setState(LiftHeightState.STARTING); try{
     * Thread.sleep(800); } catch(InterruptedException io){ io.printStackTrace(); }
     * } else { OptimusPrime.getInstance().setState(state); } return true; }
     * 
     * @Override protected void end() { }
     * 
     * }
     */
}