package org.team3128.gromit.cvcommands;

import edu.wpi.first.wpilibj.command.Command;

public class CmdAutoAim extends Command {
    private enum AutoAimState {
        SEARCHING,
        FEEDBACK,
        BLIND,
    }

    private AutoAimState aimState = AutoAimState.SEARCHING;

    public CmdAutoAim() {

    }

    @Override
    protected void initialize() {
        
    }

    @Override
    protected void execute() {
        switch (aimState) {
            case SEARCHING:
                // TODO: Move to FEEDBACK when tv reads 1 <TODO: determine what this number this actually is> times.
                break;

            case FEEDBACK:
                // TODO: Copy the SRXTankDrive FF&PID stuff here

                // TODO: Move to
                break;

            case BLIND:
                // TODO: Drive until error plateau
                break;
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        
    }

    @Override
    protected void interrupted() {
        
    }
}