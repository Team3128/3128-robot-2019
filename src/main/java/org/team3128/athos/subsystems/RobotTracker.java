/**
 * @author Adham Elarabawy 
 */
package org.team3128.athos.subsystems;

import org.team3128.common.utility.structure.CircularQueue;

import com.ctre.phoenix.Logger;

import org.team3128.common.utility.Log;
import org.team3128.common.generics.Threaded;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.math.InterpolablePair;
import org.team3128.common.utility.math.Pose2D;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.utility.math.Translation2D;

public class RobotTracker extends Threaded {

	private static final RobotTracker trackingInstance = new RobotTracker();

	public static RobotTracker getInstance() {
		return RobotTracker.trackingInstance;
	}

	private NEODrive drive;
	private Pose2D currentOdometry;
	public Pose2D trajOdometry;
	private double oldDistance = 0;
	private double currentDistance;
	private double oldTheta = 0;
	private double currentTheta;

	private RobotTracker() {
		drive = NEODrive.getInstance();
		currentOdometry = new Pose2D(new Translation2D(), drive.getGyroAngle());
		trajOdometry = new Pose2D(new Translation2D(), new Rotation2D());

	}

	// synchronized public Rotation2D getGyroAngle(long time) {
	// return gyroHistory.getInterpolatedKey(time);
	// }

	synchronized public Pose2D getOdometry() {
		return currentOdometry;
	}

	synchronized public void resetOdometry() {
		drive.resetGyro();
		currentOdometry = new Pose2D(new Translation2D(), new Rotation2D());
		oldDistance = drive.getDistance();
	}

	/**
	 * Integrates the encoders and gyro to figure out robot position. A constant
	 * curvature is assumed (aka there is a constant ratio between the velocities of
	 * the right and left sides of the drivetrain)
	 */
	@Override
	public void update() {
		currentDistance = drive.getDistance();
		currentTheta = drive.getAngle();

		double deltaDistance = currentDistance - oldDistance;

		double deltaX = deltaDistance * RobotMath.cos(currentTheta);
		double deltaY = deltaDistance * RobotMath.sin(currentTheta);
		synchronized (this) {
			double currX = currentOdometry.translationMat.getX() + deltaX;
			double currY = currentOdometry.translationMat.getY() + deltaY;

			currentOdometry = new Pose2D(new Translation2D(currX, currY), Rotation2D.fromDegrees(currentTheta));
		}
		oldDistance = currentDistance;
		oldTheta = currentTheta;
	}

	// /**
	// *
	// * @param offset
	// */
	// synchronized public void setInitialRotation(Rotation2D offset) {
	// this.rotationOffset = offset;
	// }

	// synchronized public void setInitialTranslation(Translation2D offset) {
	// this.translationOffset = offset;
	// resetOdometry();
	// }
}