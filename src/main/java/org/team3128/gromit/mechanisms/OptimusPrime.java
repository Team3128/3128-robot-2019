package org.team3128.gromit.mechanisms;

import org.team3128.gromit.mechanisms.Lift.LiftHeightState;
import org.team3128.gromit.mechanisms.LiftIntake.LiftIntakeState;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.team3128.common.autonomous.primitives.CmdRunInParallel;
import org.team3128.common.util.Log;
import org.team3128.gromit.mechanisms.LiftIntake;

import org.team3128.gromit.mechanisms.FourBar.FourBarState;
import org.team3128.gromit.mechanisms.GroundIntake.GroundIntakeState;;

/**
 * Overall mechanism wrapper to control the {@link Lift} and {@link LiftIntake}.
 * 
 * @author Chris, Jude, Tygan
 * 
 */

public class OptimusPrime {
  public enum RobotState {
      INTAKE_FLOOR_CARGO(LiftHeightState.INTAKE_FLOOR_CARGO, FourBarState.CARGO_INTAKE),
      HOLD_CARGO(LiftHeightState.HOLD_CARGO, FourBarState.CARGO_INTAKE),

      DEPOSIT_LOW_HATCH(LiftHeightState.LOW_HATCH, FourBarState.LOW),
      DEPOSIT_MID_HATCH(LiftHeightState.MID_HATCH, FourBarState.LOW),
      DEPOSIT_TOP_HATCH(LiftHeightState.TOP_HATCH, FourBarState.HIGH),

      DEPOSIT_LOW_CARGO(LiftHeightState.LOW_CARGO, FourBarState.LOW),
      DEPOSIT_MID_CARGO(LiftHeightState.MID_CARGO, FourBarState.LOW),
      DEPOSIT_TOP_CARGO(LiftHeightState.TOP_HATCH, FourBarState.HIGH),

      INTAKE_LOADING_CARGO(LiftHeightState.INTAKE_LOADING_CARGO, FourBarState.LOW);

      public LiftHeightState targetLiftState;
      public FourBarState targetFourBarState;
      //states for lift intake and ground intakes

      private RobotState(LiftHeightState liftState, FourBarState fourBarState)
      {
        this.targetLiftState = liftState;
        this.targetFourBarState = fourBarState;
      }
  }

  Lift lift;
  LiftIntake liftIntake;
  FourBar fourBar;
  GroundIntake groundIntake;

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
    groundIntake = GroundIntake.getInstance();
  }
  
  public class CmdEnterIntakeMode extends CommandGroup {
    public CmdEnterIntakeMode() {      
      addSequential(new CmdRunInParallel(
        lift.new CmdSetLiftPosition(LiftHeightState.INTAKE_FLOOR_CARGO),
        liftIntake.new CmdSetLiftIntakeState(LiftIntakeState.BALL_INTAKE)));
      addSequential(groundIntake.new CmdSetGroundIntakeState(GroundIntakeState.DEPLOYED));
      addSequential(fourBar.new CmdSetFourBarPosition(FourBarState.CARGO_INTAKE));
      addSequential(groundIntake.new CmdSetGroundIntakeState(GroundIntakeState.INTAKING));
    }
  }

  public class CmdExitIntakeMode extends CommandGroup {
    public CmdExitIntakeMode() {
      addSequential(groundIntake.new CmdSetGroundIntakeState(GroundIntakeState.DEPLOYED));
      addSequential(lift.new CmdSetLiftPosition(LiftHeightState.HOLD_CARGO));
      addSequential(groundIntake.new CmdSetGroundIntakeState(GroundIntakeState.RETRACTED));
    }
  }

  public class CmdDepositGameElement extends CommandGroup {
    public CmdDepositGameElement() {
      //TODO
    }
  }
}