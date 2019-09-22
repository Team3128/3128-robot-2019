package org.team3128.common.hardware.gyroscope;

/**
 * Interface to define a gyro sensor, which has minimal methods that allow for
 * the setting and retrieval of yaw.
 * 
 * Takes the convention of counter-clockwise rotation as positive.
 * 
 * @author Ronak Roy
 */
public interface Gyro {
    /**
     * Gets the current yaw of the robot, with counterclockwise as positive. The
     * function is continuous, so it ranges from negative infinity to positive
     * infinity.
     * 
     * @return robot yaw, in degrees
     */
    public double getAngle();

    /**
     * Gets the rate of change of the yaw of the robot, with counterclockwise
     * rotations as positive.
     * 
     * @return angular velocity, in degrees/second
     */
    public double getRate();

    /**
     * Gets the pitch of the robot in degrees
     * 
     * @return robot pitch, in degrees
     */
    public double getPitch();

    public double getRoll();

    /**
     * Gives the gyro reading an offset such that the current position is read as 0
     * degrees.
     */
    public void reset();

    /**
     * Gives the gyro reading an offset such that the current position is read as
     * whatever desired angle.
     * 
     * @param angle - The angle to consider the current position as.
     */
    public void setAngle(double angle);
}