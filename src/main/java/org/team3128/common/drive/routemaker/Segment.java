package org.team3128.common.drive.routemaker;

import org.team3128.common.util.RobotMath;

/**
 * Represents a single quintic spline that makes up part of the entire
 * parameterized trajectory.
 * 
 * @author Ronak
 *
 */
public class Segment {
	private Waypoint wp0, wp1;

	private double x0,   x1;
	private double x0p,  x1p;
	private double x0pp, x1pp;

	private double y0,   y1;
	private double y0p,  y1p;
	private double y0pp, y1pp;
	
	private double ax, bx, cx, dx, ex, fx;
	private double ay, by, cy, dy, ey, fy;

	/**
	 * Makes a new {@link Segment} object that goes from a starting {@link Waypoint} to an ending waypoint.
	 *
	 * @param wp0
	 * @param wp1
	 */
	public Segment(Waypoint wp0, Waypoint wp1) {
		this(wp0, wp1, 0);
	}

	/**
	 * Makes a new {@link Segment} object that goes from a starting {@link Waypoint} to an ending waypoint. Allows
	 * for generation of intermediate path, with the parameter s that waypoint wp0 was derived specified as n.
	 * @param wp0
	 * @param wp1
	 * @param n
	 */
	public Segment(Waypoint wp0, Waypoint wp1, double n) {
		this.wp0 = wp0;
		this.wp1 = wp1;
		
		x0 = wp0.x;
		x1 = wp1.x;
		
		x0p = wp0.fdm * RobotMath.cos(wp0.angle);
		x1p = wp1.fdm * RobotMath.cos(wp1.angle);

		x0pp = wp0.xpp;
		x1pp = wp1.xpp;
		
//		ax = 0.5 * ( 12 * x1 -  6 * x1p +     x1pp   - 12 * x0 -  6 * x0p -     x0pp);
//		bx = 0.5 * (-30 * x1 + 14 * x1p - 2 * x1pp   + 30 * x0 + 16 * x0p + 3 * x0pp);
//		cx = 0.5 * ( 20 * x1 -  8 * x1p +     x1pp   - 20 * x0 - 12 * x0p - 3 * x0pp);
//		dx = 0.5 * x0pp;
//		ex = x0p;
//		fx = x0;

		ax = -(x1pp + 12*x1 - 6*x1p - 12*x0 - 6*x0p - x0pp - 2*x1pp*n + 6*n*x1p + 6*n*x0p + 2*n*x0pp + x1pp*n*n - n*n*x0pp)/(2*(n - 1)*(n-1)*(n*n*n - 3*n*n + 3*n - 1));
		bx =  (2*x1pp + 30*x1 - 14*x1p - 30*x0 - 16*x0p - 3*x0pp - x1pp*n + 30*x1*n - 2*n*x1p - 30*n*x0 + 2*n*x0p + 4*n*x0pp - 4*x1pp*n*n + 3*x1pp*n*n*n + 16*n*n*x1p + 14*n*n*x0p + n*n*x0pp - 2*n*n*n*x0pp)/(2*(n - 1)*(n-1)*(n*n*n - 3*n*n + 3*n - 1));
		cx = -(x1pp + 20*x1 - 8*x1p - 20*x0 - 12*x0p - 3*x0pp + 4*x1pp*n + 80*x1*n - 32*n*x1p - 80*n*x0 - 28*n*x0p - 8*x1pp*n*n + 3*x1pp*n*n*n*n + 20*x1*n*n + 28*n*n*x1p + 12*n*n*n*x1p - 20*n*n*x0 + 32*n*n*x0p + 8*n*n*n*x0p + 8*n*n*x0pp - 4*n*n*n*x0pp - n*n*n*n*x0pp)/(2*(n - 1)*(n-1)*(n*n*n - 3*n*n + 3*n - 1));
		dx = -(x0pp - 3*x1pp*n - 60*x1*n + 24*n*x1p + 60*n*x0 + 36*n*x0p + 4*n*x0pp + 8*x1pp*n*n*n - 4*x1pp*n*n*n*n - x1pp*n*n*n*n*n - 60*x1*n*n + 12*n*n*x1p - 36*n*n*n*x1p + 60*n*n*x0 - 12*n*n*x0p - 24*n*n*n*x0p - 8*n*n*x0pp + 3*n*n*n*n*x0pp)/(2*(n*n - 2*n + 1)*(n*n*n - 3*n*n + 3*n - 1));
		ex = -(2*x0p - 10*n*x0p - 2*n*x0pp + 3*x1pp*n*n - 4*x1pp*n*n*n - x1pp*n*n*n*n + 2*x1pp*n*n*n*n*n + 60*x1*n*n - 24*n*n*x1p + 16*n*n*n*x1p + 10*n*n*n*n*x1p - 2*n*n*n*n*n*x1p - 60*n*n*x0 - 16*n*n*x0p + 24*n*n*n*x0p + n*n*x0pp + 4*n*n*n*x0pp - 3*n*n*n*n*x0pp)/(2*(n - 1)*(n-1)*(n*n*n - 3*n*n + 3*n - 1));
		fx = -(2*x0 - 10*n*x0 - 2*n*x0p - x1pp*n*n*n + 2*x1pp*n*n*n*n - x1pp*n*n*n*n*n - 20*x1*n*n*n + 10*x1*n*n*n*n - 2*x1*n*n*n*n*n + 8*n*n*n*x1p - 10*n*n*n*n*x1p + 2*n*n*n*n*n*x1p + 20*n*n*x0 + 10*n*n*x0p - 8*n*n*n*x0p + n*n*x0pp - 2*n*n*n*x0pp + n*n*n*n*x0pp)/(2*(n*n - 2*n + 1)*(n*n*n - 3*n*n + 3*n - 1));


		y0 = wp0.y;
		y1 = wp1.y;
		
		y0p = wp0.fdm * RobotMath.sin(wp0.angle);
		y1p = wp1.fdm * RobotMath.sin(wp1.angle);

		y0pp = wp0.ypp;
		y1pp = wp1.ypp;

//		ay = 0.5 * ( 12 * y1 -  6 * y1p +     y1pp   - 12 * y0 -  6 * y0p -     y0pp);
//		by = 0.5 * (-30 * y1 + 14 * y1p - 2 * y1pp   + 30 * y0 + 16 * y0p + 3 * y0pp);
//		cy = 0.5 * ( 20 * y1 -  8 * y1p +     y1pp   - 20 * y0 - 12 * y0p - 3 * y0pp);
//		dy = 0.5 * y0pp;
//		ey = y0p;
//		fy = y0;

		ay = -(y1pp + 12*y1 - 6*y1p - 12*y0 - 6*y0p - y0pp - 2*y1pp*n + 6*n*y1p + 6*n*y0p + 2*n*y0pp + y1pp*n*n - n*n*y0pp)/(2*(n - 1)*(n-1)*(n*n*n - 3*n*n + 3*n - 1));
		by =  (2*y1pp + 30*y1 - 14*y1p - 30*y0 - 16*y0p - 3*y0pp - y1pp*n + 30*y1*n - 2*n*y1p - 30*n*y0 + 2*n*y0p + 4*n*y0pp - 4*y1pp*n*n + 3*y1pp*n*n*n + 16*n*n*y1p + 14*n*n*y0p + n*n*y0pp - 2*n*n*n*y0pp)/(2*(n - 1)*(n-1)*(n*n*n - 3*n*n + 3*n - 1));
		cy = -(y1pp + 20*y1 - 8*y1p - 20*y0 - 12*y0p - 3*y0pp + 4*y1pp*n + 80*y1*n - 32*n*y1p - 80*n*y0 - 28*n*y0p - 8*y1pp*n*n + 3*y1pp*n*n*n*n + 20*y1*n*n + 28*n*n*y1p + 12*n*n*n*y1p - 20*n*n*y0 + 32*n*n*y0p + 8*n*n*n*y0p + 8*n*n*y0pp - 4*n*n*n*y0pp - n*n*n*n*y0pp)/(2*(n - 1)*(n-1)*(n*n*n - 3*n*n + 3*n - 1));
		dy = -(y0pp - 3*y1pp*n - 60*y1*n + 24*n*y1p + 60*n*y0 + 36*n*y0p + 4*n*y0pp + 8*y1pp*n*n*n - 4*y1pp*n*n*n*n - y1pp*n*n*n*n*n - 60*y1*n*n + 12*n*n*y1p - 36*n*n*n*y1p + 60*n*n*y0 - 12*n*n*y0p - 24*n*n*n*y0p - 8*n*n*y0pp + 3*n*n*n*n*y0pp)/(2*(n*n - 2*n + 1)*(n*n*n - 3*n*n + 3*n - 1));
		ey = -(2*y0p - 10*n*y0p - 2*n*y0pp + 3*y1pp*n*n - 4*y1pp*n*n*n - y1pp*n*n*n*n + 2*y1pp*n*n*n*n*n + 60*y1*n*n - 24*n*n*y1p + 16*n*n*n*y1p + 10*n*n*n*n*y1p - 2*n*n*n*n*n*y1p - 60*n*n*y0 - 16*n*n*y0p + 24*n*n*n*y0p + n*n*y0pp + 4*n*n*n*y0pp - 3*n*n*n*n*y0pp)/(2*(n - 1)*(n-1)*(n*n*n - 3*n*n + 3*n - 1));
		fy = -(2*y0 - 10*n*y0 - 2*n*y0p - y1pp*n*n*n + 2*y1pp*n*n*n*n - y1pp*n*n*n*n*n - 20*y1*n*n*n + 10*y1*n*n*n*n - 2*y1*n*n*n*n*n + 8*n*n*n*y1p - 10*n*n*n*n*y1p + 2*n*n*n*n*n*y1p + 20*n*n*y0 + 10*n*n*y0p - 8*n*n*n*y0p + n*n*y0pp - 2*n*n*n*y0pp + n*n*n*n*y0pp)/(2*(n*n - 2*n + 1)*(n*n*n - 3*n*n + 3*n - 1));
	}

	public Waypoint getStart() {
		return wp0;
	}

	public Waypoint getEnd() {
		return wp1;
	}


	public double getX(double s) {
		return RobotMath.polynomial(s, ax, bx, cx, dx, ex, fx);
	}

	public double getY(double s) {
		return RobotMath.polynomial(s, ay, by, cy, dy, ey, fy);
	}


	private double getXp(double s) {
		return RobotMath.polynomial(s, 5 * ax, 4 * bx, 3 * cx, 2 * dx, ex);
	}

	private double getYp(double s) {
		return RobotMath.polynomial(s, 5 * ay, 4 * by, 3 * cy, 2 * dy, ey);
	}


	private double getXpp(double s) {
		return RobotMath.polynomial(s, 20 * ax, 12 * bx, 6 * cx, 2 * dx);
	}

	private double getYpp(double s) {
		return RobotMath.polynomial(s, 20 * ay, 12 * by, 6 * cy, 2 * dy);
	}

	public Waypoint getIntermediateWaypoint(double s) {
		return new Waypoint(getX(s), getY(s), getAngle(s), getFDM(s), getXpp(s), getYpp(s));
	}

	public double getAngle(double s) {
		double dxds = getXp(s);
		double dyds = getYp(s);

		double angle = Math.toDegrees(Math.atan(dyds / dxds));

		if (dxds == 0) {
			if (dyds > 0) return 90;
			else if (dyds < 0) return 270;
			else System.out.println("For some strange reason, your robot has a point where it isn't moving; this is all good and fine, but it breaks the math. Consider splitting your path.");
		}

		if (dyds == 0) {
			if (dxds > 0) return 0;
			else if (dxds < 0) return 180;
			//I would be very surprised if it missed the wp0 fatal error...
		}

		if (dyds > 0 && dxds > 0) {
			// This is:      Quadrant 1
			// Math returns: Quadrant 1 (positive)
		}
		else if (dyds > 0 && dxds < 0) {
			// This is:      Quadrant 2
			// Math returns: Quadrant 4 (negative)
			angle += 180;
		}
		else if (dyds < 0 && dxds < 0) {
			// This is:      Quadrant 3
			// Math returns: Quadrant 1 (positive)
			angle += 180;
		}
		else if (dyds < 0 && dxds > 0) {
			// This is:      Quadrant 4
			// Math returns: Quadrant 4 (negative)
			angle += 360;
		}

		return angle;
	}

	public double getFDM(double s) {
		return Math.sqrt(RobotMath.square(getXp(s)) + RobotMath.square(getYp(s)));
	}

	public String toString() {
		return "(" +
		ax + "t^5+" + bx + "t^4+" + cx + "t^3+" + dx + "t^2+" + ex + "t+" + fx +
		"," +
		ay + "t^5+" + by + "t^4+" + cy + "t^3+" + dy + "t^2+" + ey + "t+" + fy +
		")";
	}
}