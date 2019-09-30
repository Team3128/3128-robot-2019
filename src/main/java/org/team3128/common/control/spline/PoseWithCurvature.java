package org.team3128.common.control.spline;

import org.team3128.common.utility.math.Pose2D;

/**
 * Represents a pair of a pose and a curvature.
 */
@SuppressWarnings("MemberName")
public class PoseWithCurvature {
  // Represents the pose.
  public Pose2D poseMeters;

  // Represents the curvature.
  public double curvatureRadPerMeter;

  /**
   * Constructs a PoseWithCurvature.
   *
   * @param poseMeters           The pose.
   * @param curvatureRadPerMeter The curvature.
   */
  public PoseWithCurvature(Pose2D poseMeters, double curvatureRadPerMeter) {
    this.poseMeters = poseMeters;
    this.curvatureRadPerMeter = curvatureRadPerMeter;
  }

  /**
   * Constructs a PoseWithCurvature with default values.
   */
  public PoseWithCurvature() {
    poseMeters = new Pose2D();
  }
}
