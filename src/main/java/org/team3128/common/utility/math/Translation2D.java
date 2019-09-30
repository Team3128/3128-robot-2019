/**
 * @author Adham Elarabawy
 */
package org.team3128.common.utility.math;

/**
 * Stores an x and y value. Rotates and Translates of objects returns a new
 * object.
 */
public class Translation2D implements Interpolable<Translation2D> {

	public static Translation2D fromAngleDistance(double distance, Rotation2D angle) {
		return new Translation2D(angle.sin() * distance, angle.cos() * distance);
	}

	private double x;
	private double y;

	public Translation2D() {
		x = 0;
		y = 0;
	}

	public Translation2D(double x, double y) {
		this.x = x;
		this.y = y;
	}

	/**
	 * Get the angle from offset to this Translation2D. This is done by making the
	 * offset the origin and finding the angle to this point. Then the angle to the
	 * Y axis is found from this angle. The points are treated as a vector and the
	 * direction is taken off from it. The coordinates of the points correspond to
	 * the unit circle.
	 *
	 * @param offset Point that becomes the new origin for the other point
	 * @return Angle to the Y axis from the angle from the offset to this point
	 */
	public Rotation2D getAngleFromOffsetFromYAxis(Translation2D offset) {
		return offset.getAngleFromYAxis(this);
	}

	/**
	 * Get the angle from offset to this Translation2D. This is done by making the
	 * offset the origin and finding the angle to this point. The points are treated
	 * as a vector and the direction is taken off from it. The coordinates of the
	 * points correspond to the unit circle.
	 *
	 * @param offset Point that becomes the origin for the other point
	 * @return Angle from the offset to this point
	 */
	public Rotation2D getAngleFromOffset(Translation2D offset) {
		return offset.getAngle(this);
	}

	/**
	 * Get the angle from this point to another point. Then the angle to the Y axis
	 * is found from this angle. The points are treated as a vector and the
	 * direction is taken off from it. The coordinates of the points correspond to
	 * the unit circle.
	 *
	 * @param nextPoint Point to find angle to from this point
	 * @return Angle of the two points.
	 */
	public Rotation2D getAngleFromYAxis(Translation2D nextPoint) {
		double angleOffset = Math.asin((x - nextPoint.getX()) / getDistanceTo(nextPoint));
		return Rotation2D.fromRadians(angleOffset);
	}

	/**
	 * Get the angle from this point to another point. The points are treated as a
	 * vector and the direction is taken off from it. The coordinates of the points
	 * correspond to the unit circle.
	 *
	 * @param nextPoint Point to find angle to from this point.
	 * @return Angle of the two points.
	 */
	public Rotation2D getAngle(Translation2D nextPoint) {
		double angleOffset = Math.atan2(nextPoint.getY() - y, nextPoint.getX() - x);
		return Rotation2D.fromRadians(angleOffset);
	}

	/**
	 * Get the distance between the this point and the point specified in the
	 * argument.
	 *
	 * @param nextPoint Point to find distance to.
	 * @return Distance between this point and the specified point.
	 */
	public double getDistanceTo(Translation2D nextPoint) {
		return Math.sqrt(Math.pow((x - nextPoint.getX()), 2) + Math.pow(y - nextPoint.getY(), 2));
	}

	/**
	 * Adds two translations in 2d space and returns the sum. This is similar to
	 * vector addition.
	 *
	 * <p>
	 * For example, Translation2d{1.0, 2.5} + Translation2d{2.0, 5.5} =
	 * Translation2d{3.0, 8.0}
	 *
	 * @param other The translation to add.
	 * @return The sum of the translations.
	 */
	public Translation2D plus(Translation2D other) {
		return new Translation2D(this.x + other.x, this.y + other.y);
	}

	/**
	 * Subtracts the other translation from the other translation and returns the
	 * difference.
	 *
	 * <p>
	 * For example, Translation2d{5.0, 4.0} - Translation2d{1.0, 2.0} =
	 * Translation2d{4.0, 2.0}
	 *
	 * @param other The translation to subtract.
	 * @return The difference between the two translations.
	 */
	public Translation2D minus(Translation2D other) {
		return new Translation2D(x - other.x, y - other.y);
	}

	/**
	 * Returns the inverse of the current translation. This is equivalent to
	 * rotating by 180 degrees, flipping the point over both axes, or simply
	 * negating both components of the translation.
	 *
	 * @return The inverse of the current translation.
	 */
	public Translation2D unaryMinus() {
		return new Translation2D(-x, -y);
	}

	/**
	 * Multiplies the translation by a scalar and returns the new translation.
	 *
	 * <p>
	 * For example, Translation2d{2.0, 2.5} * 2 = Translation2d{4.0, 5.0}
	 *
	 * @param scalar The scalar to multiply by.
	 * @return The scaled translation.
	 */
	public Translation2D times(double scalar) {
		return new Translation2D(x * scalar, y * scalar);
	}

	/**
	 *
	 * @return X value of this object.
	 */
	public double getX() {
		return x;
	}

	/**
	 *
	 * @return Y value of this object.
	 */
	public double getY() {
		return y;
	}

	/**
	 *
	 * @return Returns a Translation2D that when translated with this Translation2D
	 *         becomes 0, 0. Essentially the negative x and y of this Translation2D.
	 */
	public Translation2D inverse() {
		return new Translation2D(-x, -y);
	}

	/**
	 * Multiplies this point with a specified rotation matrix
	 *
	 * @param rotationMat Rotation2Dmatrix to multiply point with
	 * @return Rotated point
	 */
	public Translation2D rotateBy(Rotation2D rotationMat) {
		double x2 = x * rotationMat.cos() - y * rotationMat.sin();
		double y2 = x * rotationMat.sin() + y * rotationMat.cos();
		return new Translation2D(x2, y2);
	}

	/**
	 * Translation this point by another point.
	 *
	 * @param delta Translation2D to change this point by
	 * @return Translated point
	 */
	public Translation2D translateBy(Translation2D delta) {
		return new Translation2D(x + delta.getX(), y + delta.getY());
	}

	@Override
	public Translation2D interpolate(Translation2D other, double percentage) {
		Translation2D delta = new Translation2D(this.getX() - other.getX(), this.getY() - other.getY());
		return new Translation2D(this.getX() + delta.getX() * percentage, this.getY() + delta.getY() * percentage);
	}

	public double getMagnitude() {
		return Math.sqrt(this.getX() * this.getX() + this.getY() * this.getY());
	}

	public Translation2D getUnitVector() {
		return new Translation2D(this.getX() / getMagnitude(), this.getY() / getMagnitude());
	}

	public Translation2D scale(double d) {
		return new Translation2D(this.getX() * d, this.getY() * d);
	}

	@Override
	public String toString() {
		return ("<" + this.getX() + ", " + this.getY() + ">");
	}

}