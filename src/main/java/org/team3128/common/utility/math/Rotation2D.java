/**
 * @author Adham Elarabawy 
 */
package org.team3128.common.utility.math;

/**
 * Stores a cos and sin that is used like a rotation matrix
 */
public class Rotation2D implements Interpolable<Rotation2D> {

	/**
	 * Gets a Rotation2D from a specified degree
	 *
	 * @param angle Angle to turn into a rotation matrix
	 * @return Rotation2D from specified angle in argument
	 */
	public static Rotation2D fromDegrees(double angle) {
		return Rotation2D.fromRadians(Math.toRadians(angle));
	}

	/**
	 * Gets a Rotation2Dmatrix from a specified radian
	 *
	 * @param radians Radian to turn into a rotation matrix
	 * @return Rotation2D from specified radian in argument
	 */
	public static Rotation2D fromRadians(double radians) {
		return new Rotation2D(Math.cos(radians), Math.sin(radians));
	}

	private double cos;

	private double sin;

	public Rotation2D() {
		cos = 1;
		sin = 0;
	}

	public Rotation2D(double cos, double sin) {
		this.cos = cos;
		this.sin = sin;
	}

	public Rotation2D(double cos, double sin, boolean normalize) {
		this.cos = cos;
		this.sin = sin;
		if (normalize) {
			normalize();
		}
	}

	/**
	 *
	 * @return The cosine of this Rotation
	 */
	public double cos() {
		return cos;
	}

	/**
	 *
	 * @return The rotation matrix in degrees
	 */
	public double getDegrees() {
		return Math.toDegrees(getRadians());
	}

	/**
	 *
	 * @return The rotation matrix in radians
	 */
	public double getRadians() {
		return Math.atan2(sin, cos);
	}

	// TODO: make it work
	@Override
	public Rotation2D interpolate(Rotation2D other, double percentage) {
		Rotation2D diff = inverse().rotateBy(other);
		return rotateBy(Rotation2D.fromRadians(diff.getRadians() * percentage));
	}

	/**
	 *
	 * @return The Rotation2D that when rotated with this Rotation2Dmoves the cos to
	 *         1 and the sin to 0
	 */
	public Rotation2D inverse() {
		return new Rotation2D(cos, -sin);
	}

	/**
	 *
	 * @return The Rotation2D that is flipped about the x and y axis
	 */
	public Rotation2D flip() {
		return new Rotation2D(-cos, -sin);
	}

	/**
	 * Makes magnitude 1
	 */
	public void normalize() {
		double magnitude = Math.hypot(cos, sin);
		if (magnitude > 1E-9) {
			cos /= magnitude;
			sin /= magnitude;
		} else {
			cos = 1;
			sin = 0;
		}
	}

	/**
	 * Subtracts the new rotation from the current rotation and returns the new
	 * rotation.
	 *
	 * <p>
	 * For example, Rotation2d.fromDegrees(10) - Rotation2d.fromDegrees(100) =
	 * Rotation2d{-pi/2}
	 *
	 * @param other The rotation to subtract.
	 * @return The difference between the two rotations.
	 */
	public Rotation2D minus(Rotation2D other) {
		return rotateBy(other.unaryMinus());
	}

	/**
	 * Adds two rotations together, with the result being bounded between -pi and
	 * pi.
	 *
	 * <p>
	 * For example, Rotation2d.fromDegrees(30) + Rotation2d.fromDegrees(60) =
	 * Rotation2d{-pi/2}
	 *
	 * @param other The rotation to add.
	 * @return The sum of the two rotations.
	 */
	public Rotation2D plus(Rotation2D other) {
		return rotateBy(other);
	}

	/**
	 * Takes the inverse of the current rotation. This is simply the negative of the
	 * current angular value.
	 *
	 * @return The inverse of the current rotation.
	 */
	public Rotation2D unaryMinus() {
		return this.times(-1);
	}

	/**
	 * Multiplies the current rotation by a scalar.
	 *
	 * @param scalar The scalar.
	 * @return The new scaled Rotation2d.
	 */
	public Rotation2D times(double scalar) {
		return Rotation2D.fromDegrees(this.getDegrees() * scalar);
	}

	/**
	 *
	 * @param rotationMat Multiply this Rotation2D by this specified Rotation
	 * @return The multiplied Rotation
	 */
	public Rotation2D rotateBy(Rotation2D rotationMat) {
		return new Rotation2D(cos * rotationMat.cos() - sin * rotationMat.sin(),
				sin * rotationMat.cos() + cos * rotationMat.sin(), true);
	}

	/**
	 *
	 * @return The sin of this Rotation
	 */
	public double sin() {
		return sin;
	}

	public Translation2D getUnitVector() {
		return new Translation2D(cos, sin).getUnitVector();
	}

	public String toString() {
		return "" + getDegrees();
	}
}