/**
 * @author Adham Elarabawy 
 */
package org.team3128.common.utility.math;

/**
 * Stores a Translation2D and a Rotation2D
 */
public class Pose2D implements Interpolable<Pose2D> {

	public Rotation2D rotationMat;
	public Translation2D translationMat;

	public Pose2D() {
		rotationMat = new Rotation2D();
		translationMat = new Translation2D();
	}

	public Pose2D(double x, double y, Rotation2D rotation) {
		rotationMat = rotation;
		translationMat = new Translation2D(x, y);
	}

	/**
	 * 
	 * @param x
	 * @param y
	 * @param angle IN DEGREES
	 */
	public Pose2D(double x, double y, double angle) {
		rotationMat = Rotation2D.fromDegrees(angle);
		translationMat = new Translation2D(x, y);
	}

	public Pose2D(Translation2D translation, Rotation2D rotation) {
		rotationMat = rotation;
		translationMat = translation;
	}

	public Translation2D getTranslation() {
		return this.translationMat;
	}

	public Rotation2D getRotation() {
		return this.rotationMat;
	}

	@Override
	public Pose2D interpolate(Pose2D other, double percentage) {
		return new Pose2D(this.translationMat.interpolate(other.translationMat, percentage),
				this.rotationMat.interpolate(other.rotationMat, percentage));
	}

	/**
	 * Translates delta rotated by our rotation matrix and rotates our rotation
	 * matrix by the other rotation matrix
	 *
	 * @param delta
	 *
	 * @return
	 */
	public Pose2D transform(Pose2D delta) {
		return new Pose2D(translationMat.translateBy(delta.translationMat.rotateBy(rotationMat)),
				rotationMat.rotateBy(delta.rotationMat));
	}

	/**
	 * Returns the Transform2d that maps the one pose to another.
	 *
	 * @param other The initial pose of the transformation.
	 * @return The transform that maps the other pose to the current pose.
	 */
	public Transform2D minus(Pose2D other) {
		final var pose = this.relativeTo(other);
		return new Transform2D(pose.getTranslation(), pose.getRotation());
	}

	/**
	 * Transforms the pose by the given transformation and returns the new
	 * transformed pose.
	 *
	 * <p>
	 * The matrix multiplication is as follows [x_new] [cos, -sin, 0][transform.x]
	 * [y_new] += [sin, cos, 0][transform.y] [t_new] [0, 0, 1][transform.t]
	 *
	 * @param other The transform to transform the pose by.
	 * @return The transformed pose.
	 */
	public Pose2D plus(Transform2D other) {
		return transformBy(other);
	}

	/**
	 * Transforms the pose by the given transformation and returns the new pose. See
	 * + operator for the matrix multiplication performed.
	 *
	 * @param other The transform to transform the pose by.
	 * @return The transformed pose.
	 */
	public Pose2D transformBy(Transform2D other) {
		return new Pose2D(this.translationMat.plus(other.getTranslation().rotateBy(this.rotationMat)),
				this.rotationMat.plus(other.getRotation()));
	}

	/**
	 * Returns a Twist2d that maps this pose to the end pose. If c is the output of
	 * a.Log(b), then a.Exp(c) would yield b.
	 *
	 * @param end The end pose for the transformation.
	 * @return The twist that maps this to end.
	 */
	public Twist2D log(Pose2D end) {
		final var transform = end.relativeTo(this);
		final var dtheta = transform.getRotation().getRadians();
		final var halfDtheta = dtheta / 2.0;

		final var cosMinusOne = transform.getRotation().cos() - 1;

		double halfThetaByTanOfHalfDtheta;
		if (Math.abs(cosMinusOne) < 1E-9) {
			halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
		} else {
			halfThetaByTanOfHalfDtheta = -(halfDtheta * transform.getRotation().sin()) / cosMinusOne;
		}

		Translation2D translationPart = transform.getTranslation()
				.rotateBy(new Rotation2D(halfThetaByTanOfHalfDtheta, -halfDtheta))
				.times(Math.hypot(halfThetaByTanOfHalfDtheta, halfDtheta));

		return new Twist2D(translationPart.getX(), translationPart.getY(), dtheta);
	}

	/**
	 * Returns the other pose relative to the current pose.
	 *
	 * <p>
	 * This function can often be used for trajectory tracking or pose stabilization
	 * algorithms to get the error between the reference and the current pose.
	 *
	 * @param other The pose that is the origin of the new coordinate frame that the
	 *              current pose will be converted into.
	 * @return The current pose relative to the new origin pose.
	 */
	public Pose2D relativeTo(Pose2D other) {
		var transform = new Transform2D(other, this);
		return new Pose2D(transform.getTranslation(), transform.getRotation());
	}
}
