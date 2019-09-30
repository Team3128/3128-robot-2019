package org.team3128.common.control.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.team3128.common.control.spline.PoseWithCurvature;
import org.team3128.common.control.spline.SplineHelper;
import org.team3128.common.control.spline.SplineParameterizer;
import org.team3128.common.control.spline.Spline;
import org.team3128.common.control.trajectory.constraint.TrajectoryConstraint;
import org.team3128.common.utility.math.Pose2D;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.utility.math.Transform2D;
import org.team3128.common.utility.math.Translation2D;

public final class TrajectoryGenerator {
  /**
   * Private constructor because this is a utility class.
   */
  private TrajectoryGenerator() {
  }

  /**
   * Generates a trajectory with the given waypoints and constraints.
   *
   * @param waypoints                        A vector of points that the
   *                                         trajectory must go through.
   * @param constraints                      A vector of various velocity and
   *                                         acceleration constraints.
   * @param startVelocityMetersPerSecond     The start velocity for the
   *                                         trajectory.
   * @param endVelocityMetersPerSecond       The end velocity for the trajectory.
   * @param maxVelocityMetersPerSecond       The max velocity for the trajectory.
   * @param maxAccelerationMetersPerSecondSq The max acceleration for the
   *                                         trajectory.
   * @param reversed                         Whether the robot should move
   *                                         backwards. Note that the robot will
   *                                         still move from a -> b -> ... -> z as
   *                                         defined in the waypoints.
   * @return The trajectory.
   */

  public static Trajectory generateTrajectory(List<Pose2D> waypoints, List<TrajectoryConstraint> constraints,
      double startVelocityMetersPerSecond, double endVelocityMetersPerSecond, double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq, boolean reversed) {
    final var flip = new Transform2D(new Translation2D(), Rotation2D.fromDegrees(-180.0));

    // Make theta normal for trajectory generation if path is reversed.
    final var newWaypoints = new ArrayList<Pose2D>(waypoints.size());
    for (final var point : waypoints) {
      newWaypoints.add(reversed ? point.plus(flip) : point);
    }

    var points = splinePointsFromSplines(
        SplineHelper.getQuinticSplinesFromWaypoints(newWaypoints.toArray(new Pose2D[0])));

    // After trajectory generation, flip theta back so it's relative to the
    // field. Also fix curvature.
    if (reversed) {
      for (var point : points) {
        point.poseMeters = point.poseMeters.plus(flip);
        point.curvatureRadPerMeter *= -1;
      }
    }

    return TrajectoryParameterizer.timeParameterizeTrajectory(points, constraints, startVelocityMetersPerSecond,
        endVelocityMetersPerSecond, maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq, reversed);
  }

  /**
   * Generates a trajectory with the given waypoints and constraints.
   *
   * @param start                            The starting pose for the trajectory.
   * @param waypoints                        The interior waypoints for the
   *                                         trajectory. The headings will be
   *                                         determined automatically to ensure
   *                                         continuous curvature.
   * @param end                              The ending pose for the trajectory.
   * @param constraints                      A vector of various velocity and
   *                                         acceleration constraints.
   * @param startVelocityMetersPerSecond     The start velocity for the
   *                                         trajectory.
   * @param endVelocityMetersPerSecond       The end velocity for the trajectory.
   * @param maxVelocityMetersPerSecond       The max velocity for the trajectory.
   * @param maxAccelerationMetersPerSecondSq The max acceleration for the
   *                                         trajectory.
   * @param reversed                         Whether the robot should move
   *                                         backwards. Note that the robot will
   *                                         still move from a -> b -> ... -> z as
   *                                         defined in the waypoints.
   * @return The trajectory.
   */
  public static Trajectory generateTrajectory(Pose2D start, List<Translation2D> waypoints, Pose2D end,
      List<TrajectoryConstraint> constraints, double startVelocityMetersPerSecond, double endVelocityMetersPerSecond,
      double maxVelocityMetersPerSecond, double maxAccelerationMetersPerSecondSq, boolean reversed) {
    final var flip = new Transform2D(new Translation2D(), Rotation2D.fromDegrees(-180.0));

    final var newStart = reversed ? start.plus(flip) : start;
    final var newEnd = reversed ? end.plus(flip) : end;

    var points = splinePointsFromSplines(
        SplineHelper.getCubicSplinesFromWaypoints(newStart, waypoints.toArray(new Translation2D[0]), newEnd));

    // After trajectory generation, flip theta back so it's relative to the
    // field. Also fix curvature.
    if (reversed) {
      for (var point : points) {
        point.poseMeters = point.poseMeters.plus(flip);
        point.curvatureRadPerMeter *= -1;
      }
    }

    return TrajectoryParameterizer.timeParameterizeTrajectory(points, constraints, startVelocityMetersPerSecond,
        endVelocityMetersPerSecond, maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq, reversed);
  }

  private static List<PoseWithCurvature> splinePointsFromSplines(Spline[] splines) {
    // Create the vector of spline points.
    var splinePoints = new ArrayList<PoseWithCurvature>();

    // Add the first point to the vector.
    splinePoints.add(splines[0].getPoint(0.0));

    // Iterate through the vector and parameterize each spline, adding the
    // parameterized points to the final vector.
    for (final var spline : splines) {
      var points = SplineParameterizer.parameterize(spline);

      // Append the array of poses to the vector. We are removing the first
      // point because it's a duplicate of the last point from the previous
      // spline.
      splinePoints.addAll(points.subList(1, points.size()));
    }
    return splinePoints;
  }
}
