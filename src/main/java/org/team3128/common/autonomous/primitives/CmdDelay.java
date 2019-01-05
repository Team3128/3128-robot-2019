package org.team3128.common.autonomous.primitives;

import edu.wpi.first.wpilibj.command.Command;

public class CmdDelay extends Command
{	
	double sec;
	
    public CmdDelay(double sec)
    {
    	super(sec);
    	this.sec = sec;
    }

    protected void initialize()
    {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute()
    {
    	//do nothing
    }



    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished()
    {    	
    	return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end()
    {
		
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted()
    {
    	
    }
}
