
package org.team3128.common.utility.math;

import java.util.Objects;

/**
 * A change in distance along arc since the last pose update. We can use ideas
 * from differential calculus to create new Pose2ds from a Twist2d and vise
 * versa.
 *
 * <p>
 * A Twist can be used to represent a difference between two poses.
 */
@SuppressWarnings("MemberName")
public class Twist2D {
  /**
   * Linear "dx" component.
   */
  public double dx;

  /**
   * Linear "dy" component.
   */
  public double dy;

  /**
   * Angular "dtheta" component (radians).
   */
  public double dtheta;

  public Twist2D() {
  }

  /**
   * Constructs a Twist2d with the given values.
   * 
   * @param dx     Change in x direction relative to robot.
   * @param dy     Change in y direction relative to robot.
   * @param dtheta Change in angle relative to robot.
   */
  public Twist2D(double dx, double dy, double dtheta) {
    this.dx = dx;
    this.dy = dy;
    this.dtheta = dtheta;
  }

  /**
   * Checks equality between this Twist2d and another object.
   *
   * @param obj The other object.
   * @return Whether the two objects are equal or not.
   */
  @Override
  public boolean equals(Object obj) {
    if (obj instanceof Twist2D) {
      return Math.abs(((Twist2D) obj).dx - dx) < 1E-9 && Math.abs(((Twist2D) obj).dy - dy) < 1E-9
          && Math.abs(((Twist2D) obj).dtheta - dtheta) < 1E-9;
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(dx, dy, dtheta);
  }
}
