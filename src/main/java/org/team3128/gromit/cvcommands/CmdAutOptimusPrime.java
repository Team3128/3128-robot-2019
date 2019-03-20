package org.team3128.gromit.cvcommands;

import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.util.Log;
import org.team3128.common.util.units.Length;
import org.team3128.gromit.main.MainDeepSpaceRobot.GameElement;
import org.team3128.gromit.main.MainDeepSpaceRobot.ScoreTarget;
import org.team3128.gromit.mechanisms.Lift;
import org.team3128.gromit.mechanisms.OptimusPrime;
import org.team3128.gromit.mechanisms.OptimusPrime.RobotState;
import org.team3128.gromit.util.DeepSpaceConstants;

import edu.wpi.first.wpilibj.command.Command;

public class CmdAutOptimusPrime extends Command {
    //SRXTankDrive drive;
    OptimusPrime optimusPrime;
    Lift lift;
    
    GameElement gameElement;
    ScoreTarget scoreTarget;
    double currentTY, tyThreshold;
    Limelight limelight;
    
    boolean inThreshold;
    
    public CmdAutOptimusPrime(Limelight limelight, GameElement gameElement, ScoreTarget scoreTarget) {
        this.limelight = limelight;   
        this.gameElement = gameElement;
        this.scoreTarget = scoreTarget;
        
        optimusPrime = OptimusPrime.getInstance();
        lift = Lift.getInstance();
    }
    
    @Override
    protected void initialize() {        
        if (gameElement == GameElement.CARGO) {
            tyThreshold = DeepSpaceConstants.UPPER_TY_OPTIMUS_THRESHOLD;
        }
        else {
            tyThreshold = DeepSpaceConstants.LOWER_TY_OPTIMUS_THRESHOLD;
        }

        if (gameElement == GameElement.HATCH_PANEL && (scoreTarget == ScoreTarget.CARGO_SHIP || scoreTarget == ScoreTarget.ROCKET_LOW)) {
            optimusPrime.setState(RobotState.VISION_STATE);
        }
                
        inThreshold = false;
    }
    
    @Override
    protected void execute() {
        if (lift.getCurrentHeight() > lift.heightState.targetHeight - 4 * Length.cm && limelight.hasValidTarget()) {
            currentTY = limelight.getValue("ty", 2);
        
            if (currentTY > tyThreshold) {
                Log.info("AutOptimusPrime", "Within threshold");
                optimusPrime.setState(RobotState.getOptimusState(gameElement, scoreTarget));
                
                inThreshold = true;
            }
        }
    }
    
    @Override
    protected boolean isFinished() {
        return inThreshold;
    }
    
    @Override
    protected void end() {
        Log.info("CmdAutOptimusPrime", "Command finished normally.");
    }
    
    @Override
    protected void interrupted() {
        Log.info("CmdAutOptimusPrime", "Command interrupted.");
    }
}