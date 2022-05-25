package org.team3128.gromit.commands;

import org.team3128.common.generics.Loggable;
import org.team3128.common.util.Log;
import org.team3128.gromit.main.MainDeepSpaceRobot.GameElement;
import org.team3128.gromit.main.MainDeepSpaceRobot.ScoreTarget;
import org.team3128.gromit.mechanisms.FourBar;
import org.team3128.gromit.mechanisms.Lift;
import org.team3128.gromit.mechanisms.OptimusPrime;
import org.team3128.gromit.mechanisms.OptimusPrime.OptimusPrimeTarget;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Command to direct {@link OptimusPrime} to go to the {@link OptimusPrimeTarget} that matches the game element and scoring level.
 */
public class CmdSetOptimusPrimeTarget extends Command implements Loggable {
    @Override
    public String getTag() {
        return "CmdSetOptimusPrimeTarget";
    }
    //SRXTankDrive drive;
    OptimusPrime optimusPrime;
    Lift lift;
    FourBar fourBar;

    OptimusPrimeTarget desiredTarget;

    GameElement gameElement;
    ScoreTarget scoreTarget;
    
    private boolean intakingHatchPanel;

    boolean goingUp;
    
    public CmdSetOptimusPrimeTarget(GameElement gameElement, ScoreTarget scoreTarget, boolean intakingHatchPanel, int timeoutMs) {
        super(timeoutMs / 1000.0);
                
        this.gameElement = gameElement;
        this.scoreTarget = scoreTarget;

        this.intakingHatchPanel = intakingHatchPanel;
        
        optimusPrime = OptimusPrime.getInstance();
        lift = Lift.getInstance();
        fourBar = FourBar.getInstance();
    }
    
    @Override
    protected void initialize() {
        desiredTarget = OptimusPrimeTarget.getOptimusState(gameElement, scoreTarget, intakingHatchPanel);

        optimusPrime.setTarget(desiredTarget);
                
        goingUp = desiredTarget.fourBarAngleTarget.targetAngle > fourBar.getCurrentAngle();

        Log.info(this, "Four Bar going " + (goingUp ? "up" : "down"));
    }
    
    @Override
    protected boolean isFinished() {
        if (isTimedOut()) {
            return true;
        }
        else if (desiredTarget != OptimusPrimeTarget.DEPOSIT_LOW_HATCH) {
            return true;
        }
        else {
            return fourBar.getCurrentAngle() < 0;
        }


        // if (goingUp) {
        //     return fourBar.getCurrentAngle() > desiredState.targetFourBarState.targetAngle - 10 * Angle.DEGREES;
        // }
        // else {
        //     return fourBar.getCurrentAngle() < desiredState.targetFourBarState.targetAngle + 10 * Angle.DEGREES;
        // }
    }
    
    @Override
    protected void end() {
        if (isTimedOut()) {
            Log.unusual(this, "Command timed out.");

        }
        else {
            Log.info(this, "Optimus Prime arrived.");
        }
    }
    
    @Override
    protected void interrupted() {
        Log.info(this, "Command interrupted.");
    }
}