package org.team3128.common.drive;

/**
 * Interface describing the teleop-control functions of a TankDrive
 * Used so that autonomous programs can accept both TankDrive and SRXTankDrive for movement
 * @author Jamie
 *
 */
public interface ITankDrive
{

	/**
	 * Update the motor outputs with the given control values.
	 * @param joyX horizontal control input
	 * @param joyY vertical control input
	 * @param throttle throttle control input scaled between 1 and -1 (-.8 is 10 %, 0 is 50%, 1.0 is 100%)
	 * @param fullSpeed if false, speed will be cut by half.
	 */
    public void arcadeDrive(double joyX, double joyY, double throttle, boolean fullSpeed);
    
    /**
     * Drive by providing motor powers for each side.
     * @param powL the left side power.
     * @param powR the right side power.
     */
    public void tankDrive(double powL, double powR);

    /**
     * Stop the movement of the robot.
     */
	public void stopMovement();

	
	/**
	 * Get the estimated angle that the robot has turned since the encoders were last reset, based on the relative distances of each side.
	 * 
	 * Range: [0, 360)
	 * 0 degrees is straight ahead, 90 degrees is left.
	 * @return
	 */
	public double getRobotAngle();
}
