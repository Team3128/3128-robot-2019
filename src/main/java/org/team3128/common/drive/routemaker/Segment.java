package org.team3128.common.drive.routemaker;

import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;

/**
 * Represents a single quintic spline that makes up part of the entire
 * parameterized trajectory.
 * 
 * @author Ronak
 *
 */
public class Segment {
	private Waypoint first, second;

	private double x0, x1;
	private double x0p, x1p;
	
	private double y0, y1;
	private double y0p, y1p;
	
	private double ax, bx, cx;
	private double ay, by, cy;
	
	public Segment(Waypoint first, Waypoint second, double smoothness) {	
		this.first = first;
		this.second = second;
		
		x0 = first.x;
		x1 = second.x;
		
		x0p = smoothness * RobotMath.cos(first.angle);
		x1p = smoothness * RobotMath.cos(second.angle);
		
		ax = 0.5 * (-12 * x0 -  6 * x0p + 12 * x1 -  6 * x1p);
		bx = 0.5 * ( 30 * x0 + 16 * x0p - 30 * x1 + 14 * x1p + 1);
		cx = 0.5 * (-20 * x0 - 12 * x0p + 20 * x1 -  8 * x1p - 2);
		
		
		y0 = first.y;
		y1 = second.y;
		
		y0p = smoothness * RobotMath.sin(first.angle);
		y1p = smoothness * RobotMath.sin(second.angle);
		
		ay = 0.5 * (-12 * y0 -  6 * y0p + 12 * y1 -  6 * y1p);
		by = 0.5 * ( 30 * y0 + 16 * y0p - 30 * y1 + 14 * y1p + 1);
		cy = 0.5 * (-20 * y0 - 12 * y0p + 20 * y1 -  8 * y1p - 2);
	}

	public Waypoint getStart() {
		return first;
	}

	public Waypoint getEnd() {
		return second;
	}
	
	public double getX(double s) {
		//return ax * Math.pow(s, 5) + bx * Math.pow(s, 4) + cx * Math.pow(s, 3) + 0.5 * Math.pow(s, 2) + x0p * s + x0;
		return RobotMath.polynomial(s, ax, bx, cx, 0.5, x0p, x0);
	}
	
	public double getY(double s) {
		//return ay * Math.pow(s, 5) + by * Math.pow(s, 4) + cy * Math.pow(s, 3) + 0.5 * Math.pow(s, 2) + y0p * s + y0;
		return RobotMath.polynomial(s, ay, by, cy, 0.5, y0p, y0);
	}

	public double getAngle(double s) {
		double dxds = RobotMath.polynomial(s, 5 * ax, 4 * bx, 3 * cx, 2 * 0.5, x0p);
		double dyds = RobotMath.polynomial(s, 5 * ay, 4 * by, 3 * cy, 2 * 0.5, y0p);

		double angle = Math.toDegrees(Math.atan(dyds / dxds));

		if (dxds == 0) {
			if (dyds > 0) return 90;
			else if (dyds < 0) return 270;
			else Log.fatal("Segment", "For some strange reason, your robot has a point where it isn't moving; this is all good and fine, but it breaks the math. Consider splitting your path.");
		}

		if (dyds == 0) {
			if (dxds > 0) return 0;
			else if (dxds < 0) return 180;
			//I would be very suprised if it missed the first fatal error...
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

	public String toString() {
		return "(" +
		ax + "t^5+" + bx + "t^4+" + cx + "t^3+" + "0.5t^2+" + x0p + "t+" + x0 +
		"," +
		ay + "t^5+" + by + "t^4+" + cy + "t^3+" + "0.5t^2+" + y0p + "t+" + y0 +
		")";
	}
}
