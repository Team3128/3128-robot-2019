package org.team3128.gromit.mechanisms;

import org.team3128.gromit.mechanisms.Lift.LiftHeightState;
import org.team3128.gromit.mechanisms.LiftIntake.LiftIntakeState;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.team3128.common.autonomous.primitives.CmdRunInParallel;
import org.team3128.common.util.Log;
import org.team3128.gromit.main.MainGromit.GameElement;
import org.team3128.gromit.main.MainGromit.ScoreLevel;
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
        REST(LiftHeightState.BASE, FourBarState.HIGH),

        INTAKE_FLOOR_CARGO(LiftHeightState.INTAKE_FLOOR_CARGO, FourBarState.CARGO_INTAKE),
        HOLD_CARGO(LiftHeightState.HOLD_CARGO, FourBarState.CARGO_INTAKE),
        
        DEPOSIT_LOW_HATCH(LiftHeightState.LOW_HATCH, FourBarState.ROCKET_LOW),
        DEPOSIT_MID_HATCH(LiftHeightState.MID_HATCH, FourBarState.ROCKET_LOW),
        DEPOSIT_TOP_HATCH(LiftHeightState.TOP_HATCH, FourBarState.HIGH),
        
        DEPOSIT_LOW_CARGO(LiftHeightState.LOW_CARGO, FourBarState.ROCKET_LOW),
        DEPOSIT_MID_CARGO(LiftHeightState.MID_CARGO, FourBarState.ROCKET_LOW),
        DEPOSIT_TOP_CARGO(LiftHeightState.TOP_CARGO, FourBarState.HIGH),
        
        LOADING_SHIP_CARGO(LiftHeightState.LOADING_SHIP_CARGO, FourBarState.SHIP_LOADING),
        LOADING_SHIP_HATCH(LiftHeightState.LOADING_SHIP_HATCH, FourBarState.SHIP_LOADING);
        
        public LiftHeightState targetLiftState;
        public FourBarState targetFourBarState;
        //states for lift intake and ground intakes
        
        private RobotState(LiftHeightState liftState, FourBarState fourBarState) {
            this.targetLiftState = liftState;
            this.targetFourBarState = fourBarState;
        }

        public static RobotState getOptimusState(GameElement gameElement, ScoreLevel scoreLevel) {
            if (gameElement == GameElement.CARGO) {
                switch (scoreLevel) {
                    case LOW:
                        return DEPOSIT_LOW_CARGO;
                    case MID:
                        return DEPOSIT_MID_CARGO;
                    case TOP:
                        return DEPOSIT_TOP_CARGO;
                }
            }
            else {
                switch (scoreLevel) {
                    case LOW:
                        return DEPOSIT_LOW_HATCH;
                    case MID:
                        return DEPOSIT_MID_HATCH;
                    case TOP:
                        return DEPOSIT_TOP_HATCH;
                }
            }

            return REST;
        }
    }
    
    Lift lift;
    LiftIntake liftIntake;
    FourBar fourBar;
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

    public void setState(RobotState state) {
        lift.setState(state.targetLiftState);
        fourBar.setState(state.targetFourBarState);
    }
    
    public class CmdEnterIntakeMode extends CommandGroup {
        public CmdEnterIntakeMode() {      
            addSequential(new CmdRunInParallel(
            lift.new CmdHeightControl(LiftHeightState.INTAKE_FLOOR_CARGO),
            liftIntake.new CmdSetLiftIntakeState(LiftIntakeState.CARGO_INTAKE))
            );
            // addSequential(groundIntake.new CmdSetGroundIntakeState(GroundIntakeState.DEPLOYED));
            addSequential(fourBar.new CmdAngleControl(FourBarState.CARGO_INTAKE));
            // addSequential(groundIntake.new CmdSetGroundIntakeState(GroundIntakeState.INTAKING));
        }
    }
    
    public class CmdExitIntakeMode extends CommandGroup {
        public CmdExitIntakeMode() {
            // addSequential(groundIntake.new CmdSetGroundIntakeState(GroundIntakeState.DEPLOYED));
            addSequential(new CmdRunInParallel(
            lift.new CmdHeightControl(LiftHeightState.HOLD_CARGO),
            liftIntake.new CmdSetLiftIntakeState(LiftIntakeState.CARGO_INTAKE))
            );
            // addSequential(groundIntake.new CmdSetGroundIntakeState(GroundIntakeState.RETRACTED));
        }
    }
    
    public class CmdDepositGameElement extends CommandGroup {
        public CmdDepositGameElement() {
            //TODO
        }
    }
}