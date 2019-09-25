/**
 * @author Adham Elarabawy 
 */
package org.team3128.common.control.motion;

import org.team3128.athos.subsystems.NEODrive.AutoDriveSignal;
import org.team3128.athos.subsystems.NEODrive.DriveSignal;
import org.team3128.common.control.RateLimiter;
import org.team3128.common.control.motion.Path.DrivingData;
import org.team3128.common.utility.math.RigidTransform2D;
import org.team3128.common.utility.math.Translation2D;
import org.team3128.common.utility.NarwhalUtility;

public class PurePursuitController {
	/*
	 * 1. Translation delta compared to robot 2. Find angle to path relative to
	 * robot 3. Drive towards point
	 */

	private Path robotPath;
	private boolean isReversed;
	private RateLimiter speedProfiler;

	double trackRadius;
	double minPathSpeed;
	double maxPathSpeed;
	double minLookaheadDistance;
	double maxLookaheadDistance;

	public PurePursuitController(Path robotPath, boolean isReversed, double TRACK_RADIUS, double MIN_PATH_SPEED,
			double MAX_PATH_SPEED, double MIN_LOOKAHEAD_DISTANCE, double MAX_LOOKAHEAD_DISTANCE) {
		this.robotPath = robotPath;
		this.isReversed = isReversed;
		this.trackRadius = TRACK_RADIUS;
		this.minPathSpeed = MIN_PATH_SPEED;
		this.maxPathSpeed = MAX_PATH_SPEED;
		this.minLookaheadDistance = MIN_LOOKAHEAD_DISTANCE;
		this.maxLookaheadDistance = MAX_LOOKAHEAD_DISTANCE;

		speedProfiler = new RateLimiter(100, 1000);
		if (robotPath.isEmpty()) {
		}
	}

	/**
	 * Calculates the look ahead and the desired speed for each side of the robot.
	 *
	 * @param robotPose Robot position and gyro angle.
	 * @return Speed for each side of the robot.
	 *
	 */
	@SuppressWarnings("unchecked")
	public synchronized AutoDriveSignal calculate(RigidTransform2D robotPose) {
		if (isReversed) {
			robotPose = new RigidTransform2D(robotPose.translationMat, robotPose.rotationMat.flip());
		}
		double lookAheadDist = NarwhalUtility.coercedNormalize(speedProfiler.getLatestValue(), minPathSpeed,
				maxPathSpeed, minLookaheadDistance, maxLookaheadDistance);
		DrivingData data = robotPath.getLookAheadPoint(robotPose.translationMat, lookAheadDist);
		if (data.remainingDist == 0.0) { // If robot passes point, remaining
											// distance is 0
			return new AutoDriveSignal(new DriveSignal(0, 0), true);
		}
		double robotSpeed = speedProfiler.update(data.maxSpeed, data.remainingDist);
		if (robotSpeed < 20) {
			robotSpeed = 20;
		}
		Translation2D robotToLookAhead = getRobotToLookAheadPoint(robotPose, data.lookAheadPoint);
		// System.out.println("desired point " + robotToLookAhead.getX() + ", " +
		// robotToLookAhead.getY());
		double radius;
		radius = getRadius(robotToLookAhead);
		double delta = (robotSpeed / radius);
		double deltaSpeed = trackRadius * delta;

		if (isReversed) {
			robotSpeed *= -1;
		}
		double maxSpeed = Math.abs(robotSpeed) + Math.abs(deltaSpeed);
		if (maxSpeed > maxPathSpeed) {
			robotSpeed -= Math.copySign(maxSpeed - maxPathSpeed, robotSpeed);
		}
		return new AutoDriveSignal(new DriveSignal(robotSpeed + deltaSpeed, robotSpeed - deltaSpeed), false);
	}

	private double getRadius(Translation2D robotToLookAheadPoint) {
		// Hypotenuse^2 / (2 * X)
		double radius = Math.pow(Math.hypot(robotToLookAheadPoint.getX(), robotToLookAheadPoint.getY()), 2)
				/ (2 * robotToLookAheadPoint.getY());
		return radius;
	}

	private Translation2D getRobotToLookAheadPoint(RigidTransform2D robotPose, Translation2D lookAheadPoint) {
		Translation2D lookAheadPointToRobot = robotPose.translationMat.inverse().translateBy(lookAheadPoint);
		lookAheadPointToRobot = lookAheadPointToRobot.rotateBy(robotPose.rotationMat.inverse());
		return lookAheadPointToRobot;
	}

	/**
	 * Resets the time for the speed profiler.
	 */
	public void resetTime() {
		// TODO: Big Bang
		speedProfiler.reset();
	}

}
