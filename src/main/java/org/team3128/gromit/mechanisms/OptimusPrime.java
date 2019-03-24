package org.team3128.gromit.mechanisms;

import org.team3128.gromit.mechanisms.Lift.LiftHeightState;
import org.team3128.gromit.mechanisms.LiftIntake.LiftIntakeState;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.team3128.common.autonomous.primitives.CmdRunInParallel;
import org.team3128.common.util.Log;
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
        //STARTING(LiftHeightState.STARTING, FourBarState.ROCKET_LOW),
        //INIT(LiftHeightState.INIT_BASE, FourBarState.ZERO),
        
        ZERO(LiftHeightState.BASE, FourBarState.VERTICAL),
        REST(LiftHeightState.BASE, FourBarState.CARGO_HIGH),

        INTAKE_FLOOR_CARGO(LiftHeightState.INTAKE_FLOOR_CARGO, FourBarState.CARGO_INTAKE),

        VISION_STATE(LiftHeightState.VISION, FourBarState.HATCH_LOW),
        
        DEPOSIT_LOW_HATCH(LiftHeightState.LOW_HATCH, FourBarState.HATCH_LOW),
        DEPOSIT_MID_HATCH(LiftHeightState.MID_HATCH, FourBarState.HATCH_HIGH),
        DEPOSIT_TOP_HATCH(LiftHeightState.TOP_HATCH, FourBarState.HATCH_HIGH),
        
        DEPOSIT_LOW_CARGO(LiftHeightState.LOW_CARGO, FourBarState.CARGO_LOW),
        DEPOSIT_MID_CARGO(LiftHeightState.MID_CARGO, FourBarState.CARGO_LOW),
        DEPOSIT_TOP_CARGO(LiftHeightState.TOP_CARGO, FourBarState.CARGO_HIGH),
        
        LOADING_AND_SHIP_CARGO(LiftHeightState.LOADING_SHIP_CARGO, FourBarState.CARGO_MID),
        //LOADING_AND_SHIP_HATCH(LiftHeightState.LOADING_SHIP_HATCH, FourBarState.SHIP_AND_LOADING);
        LOADING_AND_SHIP_HATCH(LiftHeightState.LOADING_SHIP_HATCH, FourBarState.HATCH_LOW);  //DEBUG
        
        public LiftHeightState targetLiftState;
        public FourBarState targetFourBarState;
        //states for lift intake and ground intakes
        
        private RobotState(LiftHeightState liftState, FourBarState fourBarState) {
            this.targetLiftState = liftState;
            this.targetFourBarState = fourBarState;
        }

        public static RobotState getOptimusState(GameElement gameElement, ScoreTarget scoreLevel) {
            if (gameElement == GameElement.CARGO) {
                switch (scoreLevel) {
                    case CARGO_SHIP:
                        return LOADING_AND_SHIP_CARGO;
                    case ROCKET_LOW:
                        return DEPOSIT_LOW_CARGO;
                    case ROCKET_MID:
                        return DEPOSIT_MID_CARGO;
                    case ROCKET_TOP:
                        return DEPOSIT_TOP_CARGO;
                }
            }
            else {
                switch (scoreLevel) {
                    case CARGO_SHIP:
                        return LOADING_AND_SHIP_HATCH;
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

    public void setState(RobotState state) {
        this.robotState = state;

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
                lift.new CmdHeightControl(LiftHeightState.BASE),
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