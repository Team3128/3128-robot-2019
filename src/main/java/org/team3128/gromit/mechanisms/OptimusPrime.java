package org.team3128.gromit.mechanisms;

import org.team3128.gromit.mechanisms.Lift.LiftState;
import org.team3128.gromit.mechanisms.LiftIntake.LiftIntakeState;

import edu.wpi.first.wpilibj.command.Command;
import org.team3128.gromit.mechanisms.LiftIntake;

import org.team3128.gromit.mechanisms.FourBar.FourBarState;
import org.team3128.gromit.mechanisms.GroundIntake.GroundIntakeState;;

/**
 * Control system for the mechanism controlling mechanisms 
 * 
 * @author Chris, Jude, Tygan
 * 
 */

public class OptimusPrime{
  public enum RobotState{
      BALL_INTAKE_LOW(Lift.LiftState.BALL_INTAKE_LOW, FourBar.FourBarState.BALL_INTAKE),
      BALL_INTAKE_HIGH(Lift.LiftState.BALL_INTAKE_HIGH, FourBar.FourBarState.BALL_INTAKE),
      BOT_HATCH(Lift.LiftState.BOT_HATCH, FourBar.FourBarState.HATCH_PICKUP),
      MID_HATCH(Lift.LiftState.MID_HATCH, FourBar.FourBarState.HATCH_PICKUP),
      TOP_HATCH(Lift.LiftState.TOP_HATCH, FourBar.FourBarState.HIGH_DROP_OFF),
      BOT_BALL(Lift.LiftState.BOT_BALL, FourBar.FourBarState.HATCH_PICKUP),
      MID_BALL(Lift.LiftState.MID_BALL, FourBar.FourBarState.HATCH_PICKUP),
      TOP_BALL(Lift.LiftState.TOP_HATCH, FourBar.FourBarState.HIGH_DROP_OFF),
      CARGO_BALL(Lift.LiftState.CARGO_BALL, FourBar.FourBarState.HATCH_PICKUP);

      public LiftState targetLiftState;
      public FourBarState targetFourBarState;
      //states for lift intake and ground intakes

      private RobotState(LiftState liftState, FourBarState fourBarState)
      {
        this.targetLiftState = liftState;
        this.targetFourBarState = fourBarState;
      }
  }

  Lift lift;
  FourBar fourBar;
  GroundIntake groundIntake;

  //constructor
	public OptimusPrime(Lift lift, FourBar fourBar, GroundIntake groundIntake) {
    this.lift = lift;
    this.fourBar = fourBar;
    this.groundIntake = groundIntake;
				
	}

  public class CmdIntakeBall extends Command 
  {
    
		public CmdIntakeBall()
		{
			super(10);
		}

		@Override
		protected void initialize()
		{
      Thread ballIntake = new Thread(()->{
        lift.setState(LiftState.BALL_INTAKE_LOW);
        groundIntake.setState(GroundIntakeState.DEPLOYED);
        fourBar.setState(FourBarState.BALL_INTAKE);
        groundIntake.setState(GroundIntakeState.DEPLOYED_INTAKE);
        groundIntake.setState(GroundIntakeState.DEPLOYED);
        lift.liftIntake.setState(LiftIntakeState.BALL_INTAKE);
        lift.setState(LiftState.BALL_INTAKE_HIGH);
        groundIntake.setState(GroundIntakeState.RETRACTED);  
      });
      ballIntake.start();
    }


		@Override
		protected void execute() {
			//Log.debug("CmdSetLiftPosition", "Error: " + (LiftMotor.getSelectedSensorPosition(0) - (int)(heightState.targetHeight * ratio)));
		}

		@Override
		protected void end() {
      lift.powerControl(0);
      fourBar.powerControl(0);
		}

		@Override
		protected void interrupted()
		{
			end();
		}

    @Override
    protected boolean isFinished() {
      return isTimedOut();
      //return isTimedOut() || Math.abs(LiftMotor.getSelectedSensorPosition(0) - (int)(heightState.targetHeight * ratio)) < 300;
    }
  }
}