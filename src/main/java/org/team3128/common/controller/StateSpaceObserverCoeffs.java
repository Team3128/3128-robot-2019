package org.team3128.common.controller;

import org.team3128.common.util.math.Matrix;
import org.team3128.common.util.math.Num;

/**
 * A container for all the observer coefficients.
 */
@SuppressWarnings({ "unused", "ClassTypeParameterName", "MemberName", "ParameterName", "LineLength" })
public final class StateSpaceObserverCoeffs<States extends Num, Inputs extends Num, Outputs extends Num> {
  /**
   * Estimator gain matrix.
   */
  private final Matrix<States, Outputs> K;

  /**
   * Construct the container for the observer coefficients.
   *
   * @param K The Kalman gain matrix.
   */
  public StateSpaceObserverCoeffs(Matrix<States, Outputs> K) {
    this.K = K;
  }

  public Matrix<States, Outputs> getK() {
    return K;
  }
}
