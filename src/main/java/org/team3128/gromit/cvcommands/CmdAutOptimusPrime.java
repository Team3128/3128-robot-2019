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

    Limelight limelight;

    GameElement gameElement;
    ScoreTarget scoreTarget;
    
    private double targetHeight;
    private boolean intakingHatchPanel;

    private boolean visionStating;

    private double approximateDistance;
    boolean inThreshold;
    
    public CmdAutOptimusPrime(Limelight limelight, GameElement gameElement, ScoreTarget scoreTarget, boolean intakingHatchPanel) {
        this.limelight = limelight;

        targetHeight = DeepSpaceConstants.getVisionTargetHeight(gameElement, scoreTarget);
        
        this.gameElement = gameElement;
        this.scoreTarget = scoreTarget;

        this.intakingHatchPanel = intakingHatchPanel;
        
        optimusPrime = OptimusPrime.getInstance();
        lift = Lift.getInstance();
    }
    
    @Override
    protected void initialize() {
        visionStating = !intakingHatchPanel && gameElement == GameElement.HATCH_PANEL && (scoreTarget == ScoreTarget.CARGO_SHIP || scoreTarget == ScoreTarget.ROCKET_LOW);

        if (visionStating) {
            optimusPrime.setState(RobotState.VISION_STATE);
        }
        else {
            optimusPrime.setState(RobotState.getOptimusState(gameElement, scoreTarget));
        }
                
        inThreshold = false;
    }
    
    @Override
    protected void execute() {
        if (limelight.hasValidTarget()) {
            if (!visionStating || lift.getCurrentHeight() - lift.heightState.targetHeight > -2 * Length.in) {
                approximateDistance = limelight.getApproximateDistance(targetHeight, 2);
        
                if (approximateDistance < DeepSpaceConstants.AUTOPTIMUS_DISTANCE) {
                    Log.info("AutOptimusPrime", "Reached threshold distance.");
                    if(visionStating){
                        limelight.turnOffLED();
                    }
                    optimusPrime.setState(RobotState.getOptimusState(gameElement, scoreTarget));
                    
                    inThreshold = true;
                }
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