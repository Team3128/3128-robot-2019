/**
 * @author Adham Elarabawy 
 */
package org.team3128.common.control.motion;

import org.team3128.athos.subsystems.NEODrive.AutoDriveSignal;
import org.team3128.athos.subsystems.NEODrive.DriveSignal;
import org.team3128.common.control.RateLimiter;
import org.team3128.common.control.trajectory.Trajectory;
import org.team3128.common.control.trajectory.Trajectory.State;
import org.team3128.common.utility.math.Pose2D;
import org.team3128.common.utility.math.Translation2D;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.NarwhalUtility;
import org.team3128.athos.subsystems.Constants;
import org.team3128.athos.subsystems.RobotTracker;

public class RamseteController {

	double b;
	double zeta;

	boolean isReversed = false;

	double trackRadius;

	public RamseteController(double b, double zeta, boolean isReversed, double TRACK_RADIUS) {
		this.b = b;
		this.zeta = zeta;
		this.isReversed = isReversed;
		this.trackRadius = TRACK_RADIUS;

	}

	/**
	 * Calculates the desired speed for each side of the robot.
	 *
	 * @param robotPose Robot position and gyro angle.
	 * @return Speed for each side of the robot.
	 *
	 */
	@SuppressWarnings("unchecked")
	public synchronized AutoDriveSignal calculate(Pose2D robotPose, State currentTrajectoryState) {
		double setpointLinearVelocity;
		double setpointAngularVelocity;

		double actualXPosition = robotPose.getTranslation().getX() * Constants.inchesToMeters; // actual X position in
																								// meters
		double actualYPosition = robotPose.getTranslation().getY() * Constants.inchesToMeters; // actual Y position in
																								// meters
		double actualTheta = robotPose.getRotation().getRadians(); // actual theta in radians

		double desiredLinearVelocity = currentTrajectoryState.velocityMetersPerSecond; // trajectory
		// desired
		// velocity in
		// m/s

		double desiredAngularVelocity = currentTrajectoryState.velocityMetersPerSecond
				* currentTrajectoryState.curvatureRadPerMeter; // trajectory desired angular velocity in
																// rad/s
		double desiredXPosition = currentTrajectoryState.poseMeters.getTranslation().getX(); // trajectory
		// desired
		// X
		// position
		// in
		// meters
		double desiredYPosition = currentTrajectoryState.poseMeters.getTranslation().getY(); // trajectory
		// desired
		// Y
		// position
		// in
		// meters

		RobotTracker.getInstance().trajOdometry = new Pose2D(new Translation2D(desiredXPosition / Constants.inchesToMeters, desiredYPosition / Constants.inchesToMeters), new Rotation2D());
		double desiredTheta = currentTrajectoryState.poseMeters.getRotation().getRadians(); // trajectory desired theta
																							// in radians

		double deltaTheta = desiredTheta - actualTheta;
		if (deltaTheta > Math.PI) {
			deltaTheta = -(2 * Math.PI - deltaTheta);
		}
		if (deltaTheta < -Math.PI) {
			deltaTheta = 2 * Math.PI + deltaTheta;
		}
		double deltaX = desiredXPosition - actualXPosition;
		double deltaY = desiredYPosition - actualYPosition;

		double k = 2 * zeta * Math.sqrt(Math.pow(desiredAngularVelocity, 2) + b * Math.pow(desiredLinearVelocity, 2));
		double sinc = (Math.sin(deltaTheta)) / (deltaTheta);

		setpointLinearVelocity = desiredLinearVelocity * Math.cos(deltaTheta)
				+ k * ((deltaX) * Math.cos(actualTheta) + (deltaY) * Math.sin(actualTheta));
		setpointAngularVelocity = desiredAngularVelocity + b * desiredLinearVelocity * sinc
				* ((deltaY) * Math.cos(actualTheta) - (deltaX) * Math.sin(actualTheta)) + k * (deltaTheta);

		setpointLinearVelocity /= Constants.inchesToMeters;
		double rightVelocity = setpointLinearVelocity + trackRadius * setpointAngularVelocity;
		double leftVelocity = setpointLinearVelocity - trackRadius * setpointAngularVelocity;
		return new AutoDriveSignal(new DriveSignal(leftVelocity, rightVelocity), false);
	}
}
