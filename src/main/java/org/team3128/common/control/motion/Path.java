package org.team3128.common.control.motion;

import java.util.ArrayList;
import java.util.List;

import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.utility.math.Translation2D;

/**
 * Contains a list of points and generated segments from these points. This can
 * be used to calculate lookahead in this class.
 */
public class Path {

	public static class DrivingData {
		public double remainingDist, maxSpeed;
		public Translation2D lookAheadPoint;
		public Translation2D closestPoint;
		public Translation2D currentSegEnd;
	}

	private List<Segment> segments;
	private Translation2D lastPoint;
	private Rotation2D endAngle = null;
	private double totalDist;
	private double finishedDist;
	private volatile boolean isEmpty;
	private double percentage;

	double minTurningRadius;

	/**
	 * Contains a list of points and can create path segments with them. Lookahead
	 * is calculated with using this.
	 *
	 * @param start Initial point in path.
	 */
	public Path(Translation2D start, double MIN_TURNING_RADIUS) {
		this.minTurningRadius = MIN_TURNING_RADIUS;
		segments = new ArrayList<Segment>();
		lastPoint = start;
		isEmpty = true;
		totalDist = 0;
		finishedDist = 0;
	}

	/**
	 * Add another point to the path.
	 *
	 * @param x
	 * @param y
	 * @param speed
	 */
	public void addPoint(double x, double y, double speed) {
		Segment newSegment = new Segment(lastPoint.getX(), lastPoint.getY(), x, y, speed);
		segments.add(newSegment);
		totalDist += newSegment.getDistance();
		lastPoint = new Translation2D(x, y);
		isEmpty = false;
	}

	public void addPoint(Translation2D point, double speed) {
		addPoint(point.getX(), point.getY(), speed);
		isEmpty = false;
	}

	/**
	 * Sets the desired angle for the robot to end in. It does this by placing up to
	 * two points that have a 90 degree angle to allow the robot to converge on the
	 * path.
	 *
	 * @param angle Angle for the robot to end in.
	 */
	public void setAngle(Rotation2D angle) {
		endAngle = angle;
	}

	public boolean isEmpty() {
		return isEmpty;
	}

	/**
	 * Prints all the points on the path.
	 */
	synchronized public void printAllPoints() {
		for (Segment segment : segments) {
			System.out.println(segment.getStart().getX() + "    " + segment.getStart().getY());
		}
		System.out.println(segments.get(segments.size() - 1).getEnd().getX() + "   "
				+ segments.get(segments.size() - 1).getEnd().getY());
	}

	public double getPercentage() {
		return this.percentage;
	}

	/**
	 *
	 * @param pose              Current robot position
	 * @param lookAheadDistance Distance on the path to get the look ahead.
	 * @return
	 */
	public DrivingData getLookAheadPoint(Translation2D pose, double lookAheadDistance) {
		DrivingData data = new DrivingData();
		Translation2D closestPoint = segments.get(0).getClosestPoint(pose);
		Translation2D closestToRobot = closestPoint.inverse().translateBy(pose);
		// Remove old points that we have passed
		while (segments.size() > 1) {
			double distToClosest = Math.hypot(closestToRobot.getX(), closestToRobot.getY());
			Translation2D closestNextPoint = segments.get(1).getClosestPoint(pose);
			Translation2D closestNextToRobot = closestNextPoint.inverse().translateBy(pose);
			double distToNext = Math.hypot(closestNextToRobot.getX(), closestNextToRobot.getY());
			if (distToClosest > distToNext) {
				// Run commands that didn't run yet in segments being deleted
				finishedDist += segments.get(0).getDistance();
				segments.remove(0);
				closestPoint = closestNextPoint;
				closestToRobot = closestNextToRobot;
			} else {
				break;
			}
		}
		// Run commands when we zoom past
		double traveledDist = (segments.get(0).getPercentageOnSegment(pose) * segments.get(0).getDistance()
				+ finishedDist);
		double percentage = traveledDist / totalDist;
		this.percentage = percentage;
		data.closestPoint = closestPoint;
		data.currentSegEnd = segments.get(0).getEnd();
		Translation2D closestToEnd = closestPoint.inverse().translateBy(segments.get(0).getEnd());
		Translation2D closestToStart = segments.get(0).getStart().inverse().translateBy(closestPoint);

		lookAheadDistance += Math.hypot(closestToRobot.getX(), closestToRobot.getY());
		double remainingSegDist = Math.hypot(closestToEnd.getX(), closestToEnd.getY());
		data.remainingDist = remainingSegDist;
		data.maxSpeed = segments.get(0).getMaxSpeed();
		for (int i = 1; i < segments.size(); i++) {
			data.remainingDist += segments.get(i).getDistance();
		}
		// If the lookahead point lies on a path then return the point
		// else extrapolate past the end point from the last segment
		if (lookAheadDistance > remainingSegDist && segments.size() > 1) {
			lookAheadDistance -= remainingSegDist;
			for (int i = 1; i < segments.size(); i++) {
				if (lookAheadDistance > segments.get(i).getDistance() && i != (segments.size() - 1)) {
					lookAheadDistance -= segments.get(i).getDistance();
				} else {
					data.lookAheadPoint = segments.get(i).getPointByDistance(lookAheadDistance);
					break;
				}
			}
		} else {
			double distanceOnPath = segments.get(0).getDistance() - remainingSegDist + lookAheadDistance;
			data.lookAheadPoint = segments.get(0).getPointByDistance(distanceOnPath);
		}
		return data;
	}
}
