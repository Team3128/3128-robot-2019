package org.team3128.common.controller;

import org.team3128.common.util.math.Matrix;
import org.team3128.common.util.math.Num;

/**
 * A container for all the observer coefficients.
 */
@SuppressWarnings({ "ClassTypeParameterName", "MemberName", "ParameterName", "LineLength" })
public final class PeriodVariantObserverCoeffs<States extends Num, Inputs extends Num, Outputs extends Num> {
  /**
   * The continuous process noise covariance matrix.
   */
  private final Matrix<States, States> Qcontinuous;

  /**
   * The continuous measurement noise covariance matrix.
   */
  private final Matrix<Outputs, Outputs> Rcontinuous;

  /**
   * The steady-state error covariance matrix.
   */
  private final Matrix<States, States> PsteadyState;

  /**
   * Construct the container for the observer coefficients.
   *
   * @param Qcontinuous  The continuous process noise covariance matrix.
   * @param Rcontinuous  The continuous measurement noise covariance matrix.
   * @param PsteadyState The steady-state error covariance matrix.
   */
  public PeriodVariantObserverCoeffs(Matrix<States, States> Qcontinuous, Matrix<Outputs, Outputs> Rcontinuous,
      Matrix<States, States> PsteadyState) {
    this.Qcontinuous = Qcontinuous;
    this.Rcontinuous = Rcontinuous;
    this.PsteadyState = PsteadyState;
  }

  public Matrix<States, States> getQcontinuous() {
    return Qcontinuous;
  }

  public Matrix<Outputs, Outputs> getRcontinuous() {
    return Rcontinuous;
  }

  public Matrix<States, States> getPsteadyState() {
    return PsteadyState;
  }
}
