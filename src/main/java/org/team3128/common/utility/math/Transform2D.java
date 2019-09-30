
package org.team3128.common.utility.math;

import java.util.Objects;

/**
 * Represents a transformation for a Pose2d.
 */
public class Transform2D {
  private final Translation2D m_translation;
  private final Rotation2D m_rotation;

  /**
   * Constructs the transform that maps the initial pose to the final pose.
   *
   * @param initial The initial pose for the transformation.
   * @param last    The final pose for the transformation.
   */
  public Transform2D(Pose2D initial, Pose2D last) {
    // We are rotating the difference between the translations
    // using a clockwise rotation matrix. This transforms the global
    // delta into a local delta (relative to the initial pose).
    m_translation = last.getTranslation().minus(initial.getTranslation()).rotateBy(initial.getRotation().unaryMinus());

    m_rotation = last.getRotation().minus(initial.getRotation());
  }

  /**
   * Constructs a transform with the given translation and rotation components.
   *
   * @param translation Translational component of the transform.
   * @param rotation    Rotational component of the transform.
   */
  public Transform2D(Translation2D translation, Rotation2D rotation) {
    m_translation = translation;
    m_rotation = rotation;
  }

  /**
   * Constructs the identity transform -- maps an initial pose to itself.
   */
  public Transform2D() {
    m_translation = new Translation2D();
    m_rotation = new Rotation2D();
  }

  /**
   * Scales the transform by the scalar.
   *
   * @param scalar The scalar.
   * @return The scaled Transform2d.
   */
  public Transform2D times(double scalar) {
    return new Transform2D(m_translation.times(scalar), m_rotation.times(scalar));
  }

  /**
   * Returns the translation component of the transformation.
   *
   * @return The translational component of the transform.
   */
  public Translation2D getTranslation() {
    return m_translation;
  }

  /**
   * Returns the rotational component of the transformation.
   *
   * @return Reference to the rotational component of the transform.
   */
  public Rotation2D getRotation() {
    return m_rotation;
  }

  /**
   * Checks equality between this Transform2d and another object.
   *
   * @param obj The other object.
   * @return Whether the two objects are equal or not.
   */
  @Override
  public boolean equals(Object obj) {
    if (obj instanceof Transform2D) {
      return ((Transform2D) obj).m_translation.equals(m_translation)
          && ((Transform2D) obj).m_rotation.equals(m_rotation);
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(m_translation, m_rotation);
  }
}
