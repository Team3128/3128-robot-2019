package org.team3128.gromit.mechanisms;

import org.team3128.gromit.mechanisms.Lift.LiftState;
import org.team3128.gromit.mechanisms.FourBar.FourBarState;;


public class OptimusPrime{
    public enum RobotState{
    BallIntake(Lift.LiftState.BALL_INTAKE, FourBar.FourBarState.BALL_INTAKE);

    public LiftState targetLiftState;
    public FourBarState targetFourBarState;
    //states for lift intake and ground intakes

    private RobotState(LiftState liftState, FourBarState fourBarState)
		{
            this.targetLiftState = liftState;
            this.targetFourBarState = fourBarState;
		}
}
/*
so depending on driver input, the state of the robot should change to a 'goal state'.
This class will set the state of each mechanism and make sure that there is no collision 
*/
}