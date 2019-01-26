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
	public double fdm;
	public double xpp, ypp;
	
	/**
	 * @param x - The x position of the waypoint.
	 * @param y - The y position of the waypoint.
	 * @param angle - The robot's angle of orientation (in degrees) as it passes through the waypoint.
	 */
	public Waypoint(double x, double y, double angle, double fdm, double xpp, double ypp) {
		this.x = x;
		this.y = y;

		this.angle = angle;

		this.fdm = fdm;

		this.xpp = xpp;
		this.ypp = ypp;
	}

	public Waypoint(double x, double y, double angle, double fdm) {
		this(x, y, angle, fdm, 1, 1);
	}

	public Waypoint(Waypoint base, double x, double y, double angle) {
		this.x = x;
		this.y = y;

		this.angle = angle;

		this.fdm = base.fdm;

		this.xpp = base.xpp;
		this.ypp = base.ypp;
	}
}