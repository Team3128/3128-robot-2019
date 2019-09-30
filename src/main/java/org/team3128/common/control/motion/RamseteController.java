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
import org.team3128.common.utility.NarwhalUtility;
import org.team3128.common.utility.RobotMath;

public class RamseteController {

	double metersToInches = 39.3700787;
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

		double actualXPosition = robotPose.getTranslation().getX(); // actual X position in inches
		double actualYPosition = robotPose.getTranslation().getY(); // actual Y position in inches
		double actualTheta = robotPose.getRotation().getDegrees(); // actual theta in degrees

		double desiredLinearVelocity = currentTrajectoryState.velocityMetersPerSecond * metersToInches; // trajectory
																										// desired
																										// velocity in
																										// in/s
		double desiredAngularVelocity = currentTrajectoryState.velocityMetersPerSecond
				* currentTrajectoryState.curvatureRadPerMeter * 180 / Math.PI; // trajectory desired angular velocity in
																				// deg/s
		double desiredXPosition = currentTrajectoryState.poseMeters.getTranslation().getX() * metersToInches; // trajectory
																												// desired
																												// X
																												// position
																												// in
																												// inches
		double desiredYPosition = currentTrajectoryState.poseMeters.getTranslation().getY() * metersToInches; // trajectory
																												// desired
																												// Y
																												// position
																												// in
																												// inches
		double desiredTheta = currentTrajectoryState.poseMeters.getRotation().getDegrees(); // trajectory desired theta
																							// in degrees

		double deltaTheta = desiredTheta - actualTheta;
		double deltaX = desiredXPosition - actualXPosition;
		double deltaY = desiredYPosition - actualYPosition;

		double k = 2 * zeta * Math.sqrt(Math.pow(desiredAngularVelocity, 2) + b * Math.pow(desiredLinearVelocity, 2));
		double sinc = (RobotMath.sin(deltaTheta)) / (deltaTheta);

		setpointLinearVelocity = desiredLinearVelocity * RobotMath.cos(deltaTheta)
				+ k * ((deltaX) * RobotMath.cos(actualTheta) + (deltaY) * RobotMath.sin(actualTheta));
		setpointAngularVelocity = desiredAngularVelocity
				+ b * desiredLinearVelocity * sinc
						* ((deltaY) * RobotMath.cos(actualTheta) - (deltaX) * RobotMath.sin(actualTheta))
				+ k * (deltaTheta);

		double rightVelocity = setpointLinearVelocity + trackRadius * setpointAngularVelocity;
		double leftVelocity = setpointLinearVelocity - trackRadius * setpointAngularVelocity;
		return new AutoDriveSignal(new DriveSignal(leftVelocity, rightVelocity), false);
	}
}
