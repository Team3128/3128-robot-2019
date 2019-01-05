package org.team3128.common.drive.routemaker;

/**
 * Represents a point in the overall trajectory (specified by the programmer)
 * that must be passed through during the course of following the trajectory.
 * 
 * @author Ronak
 *
 */
public class Waypoint {
	public double x, y, angle;
	
	/**
	 * @param x - The x position of the waypoint.
	 * @param y - The y position of the waypoint.
	 * @param angle - The robot's angle of orientation (in degrees) as it passes through the waypoint.
	 */
	public Waypoint(double x, double y, double angle) {
		this.x = x;
		this.y = y;
		
		this.angle = angle;
	}
	
	
}
