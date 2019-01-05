package org.team3128.common.util.datatypes;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class to store PID constants.
 * @author Jamie
 *
 */
public class PIDConstants
{
	private double kP, kI, kD, kF;
	
	private boolean usingSmartDashboard = false;
	
	//SmartDashboard keys
	String kPKey, kIKey, kDKey, kFKey;
	
	
	public PIDConstants(double kP, double kI, double kD, double kF)
	{
		this.kP = kP;
		
		this.kI = kI;
		
		this.kD = kD;
		
		this.kF = kF;
	}
	
	public PIDConstants(double kP)
	{
		this.kP = kP;
		
		this.kI = 0;
		
		this.kD = 0;
		
		this.kF = 0;
	}
	
	public double getkP()
	{
		updateFromDashboard();
		return kP;
	}

	public double getkI()
	{
		updateFromDashboard();
		return kI;
	}

	public double getkD()
	{
		updateFromDashboard();
		return kD;
	}
	
	public double getkF()
	{
		updateFromDashboard();
		return kF;
	}

	/**
	 * Put these values on the SmartDashboard so they can be edited by the user.
	 * 
	 * @param name The prefix to the name that the values will have on the dashboard.  Will look like "kP for <prefix>"
	 */
	public void putOnSmartDashboard(String name)
	{
		usingSmartDashboard = true;
		
		kPKey= "kP for " + name;
		kIKey= "kI for " + name;
		kDKey= "kD for " + name;
		kFKey= "kF for " + name;

		if(SmartDashboard.containsKey(kPKey))
		{
			return;
		}
		
		SmartDashboard.putNumber(kPKey, kP);
		SmartDashboard.putNumber(kIKey, kI);
		SmartDashboard.putNumber(kDKey, kD);
		SmartDashboard.putNumber(kFKey, kF);

	}
	
	/**
	 * Reads the constants from the SmartDashboard if necessary
	 */
	private void updateFromDashboard()
	{
		if(usingSmartDashboard)
		{
			kP = SmartDashboard.getNumber(kPKey, 0);
			kI = SmartDashboard.getNumber(kIKey, 0);
			kD = SmartDashboard.getNumber(kDKey, 0);
			kF = SmartDashboard.getNumber(kFKey, 0);
		}
	}
}