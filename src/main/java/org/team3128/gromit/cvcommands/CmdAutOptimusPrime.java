package org.team3128.gromit.cvcommands;

import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.hardware.limelight.LEDMode;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.util.Log;
import org.team3128.common.util.units.Angle;
import org.team3128.common.util.units.Length;
import org.team3128.gromit.main.MainDeepSpaceRobot.GameElement;
import org.team3128.gromit.main.MainDeepSpaceRobot.ScoreTarget;
import org.team3128.gromit.mechanisms.FourBar;
import org.team3128.gromit.mechanisms.Lift;
import org.team3128.gromit.mechanisms.OptimusPrime;
import org.team3128.gromit.mechanisms.OptimusPrime.RobotState;
import org.team3128.gromit.util.DeepSpaceConstants;

import edu.wpi.first.wpilibj.command.Command;

public class CmdAutOptimusPrime extends Command {
    //SRXTankDrive drive;
    OptimusPrime optimusPrime;
    Lift lift;
    FourBar fourBar;

    RobotState desiredState;

    GameElement gameElement;
    ScoreTarget scoreTarget;
    
    private boolean intakingHatchPanel;

    boolean goingUp;
    
    public CmdAutOptimusPrime(GameElement gameElement, ScoreTarget scoreTarget, boolean intakingHatchPanel, int timeoutMs) {
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
        desiredState = RobotState.getOptimusState(gameElement, scoreTarget, intakingHatchPanel);

        optimusPrime.setState(desiredState);
                
        goingUp = desiredState.targetFourBarState.targetAngle > fourBar.getCurrentAngle();

        Log.info("CmdAutOptimusPrime", "Four Bar going " + (goingUp ? "up" : "down"));
    }
    
    @Override
    protected boolean isFinished() {
        if (isTimedOut()) {
            return true;
        }
        else if (desiredState != RobotState.DEPOSIT_LOW_HATCH) {
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
            Log.unusual("CmdAutOptimusPrime", "Command timed out.");

        }
        else {
            Log.info("CmdAutOptimusPrime", "Optimus Prime arrived.");
        }
    }
    
    @Override
    protected void interrupted() {
        Log.info("CmdAutOptimusPrime", "Command interrupted.");
    }
}