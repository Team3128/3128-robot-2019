package org.team3128.athos.subsystems;

import org.team3128.common.utility.structure.CircularQueue;
import org.team3128.common.generics.Threaded;
import org.team3128.common.utility.math.InterpolablePair;
import org.team3128.common.utility.math.RigidTransform2D;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.utility.math.Translation2D;

public class RobotTracker extends Threaded {

	private static final RobotTracker trackingInstance = new RobotTracker();

	public static RobotTracker getInstance() {
		return RobotTracker.trackingInstance;
	}

	private NEODrive drive;
	private RigidTransform2D currentOdometry;
	private CircularQueue<RigidTransform2D> vehicleHistory;
	private CircularQueue<Rotation2D> gyroHistory;

	private double currentDistance, oldDistance, deltaDistance;
	private Rotation2D rotationOffset;
	private Translation2D translationOffset;

	private RobotTracker() {
		vehicleHistory = new CircularQueue<>(100);
		gyroHistory = new CircularQueue<>(200);
		drive = NEODrive.getInstance();
		currentOdometry = new RigidTransform2D(new Translation2D(), drive.getGyroAngle());
		rotationOffset = Rotation2D.fromDegrees(0);
		translationOffset = new Translation2D();
	}

	synchronized public Rotation2D getGyroAngle(long time) {
		return gyroHistory.getInterpolatedKey(time);
	}

	synchronized public RigidTransform2D getOdometry() {
		return currentOdometry;
	}

	synchronized public void resetOdometry() {
		drive.resetGyro();
		currentOdometry = new RigidTransform2D(new Translation2D().translateBy(translationOffset),
				Rotation2D.fromDegrees(0).rotateBy(rotationOffset));
		oldDistance = drive.getDistance();
	}

	/**
	 * Integrates the encoders and gyro to figure out robot position. A constant
	 * curvature is assumed
	 */
	@Override
	public void update() {
		double leftDist = drive.getLeftDistance(); // useless rn. TODO: take this out when you confirm that
													// drive.getDistance works.
		double rightDist = drive.getRightDistance(); // useless rn. TODO: take this out when you confirm that
														// drive.getDistance works.

		// currentDistance = (leftDist + rightDist) / 2;
		currentDistance = drive.getDistance();

		deltaDistance = currentDistance - oldDistance;
		Translation2D deltaPosition = new Translation2D(deltaDistance, 0);
		Rotation2D deltaRotation = drive.getGyroAngle().inverse().rotateBy(rotationOffset);
		synchronized (this) {
			deltaRotation = currentOdometry.rotationMat.inverse().rotateBy(deltaRotation);
			Rotation2D halfRotation = Rotation2D.fromRadians(deltaRotation.getRadians() / 2.0);
			currentOdometry = currentOdometry
					.transform(new RigidTransform2D(deltaPosition.rotateBy(halfRotation), deltaRotation));
			vehicleHistory.add(new InterpolablePair<>(System.nanoTime(), currentOdometry));
			gyroHistory.add(new InterpolablePair<>(System.nanoTime(), drive.getGyroAngle()));
		}
		oldDistance = currentDistance;
		/*
		 * System.out.println("Position: " + currentOdometry.translationMat.getX() + " "
		 * + currentOdometry.translationMat.getY()); System.out.println("Gyro: " +
		 * currentOdometry.rotationMat.getDegrees());
		 */
	}

	/**
	 *
	 * @param offset
	 */
	synchronized public void setInitialRotation(Rotation2D offset) {
		this.rotationOffset = offset;
	}

	synchronized public void setInitialTranslation(Translation2D offset) {
		this.translationOffset = offset;
		resetOdometry();
	}
}