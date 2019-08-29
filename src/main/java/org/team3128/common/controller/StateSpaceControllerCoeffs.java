package org.team3128.common.controller;

import org.team3128.common.util.math.Matrix;
import org.team3128.common.util.math.Num;
import org.team3128.common.util.math.numbers.N1;

/**
 * A container for all the state-space controller coefficients.
 */
@SuppressWarnings({ "unused", "ParameterName", "ClassTypeParameterName", "MemberName", "LineLength" })
public final class StateSpaceControllerCoeffs<States extends Num, Inputs extends Num, Outputs extends Num> {
  /**
   * Controller gain matrix.
   */
  private final Matrix<Inputs, States> K;

  /**
   * Controller feedforward gain matrix.
   */
  private final Matrix<Inputs, States> Kff;

  /**
   * Minimum control input.
   */
  private final Matrix<Inputs, N1> Umin;

  /**
   * Maximum control input.
   */
  private final Matrix<Inputs, N1> Umax;

  /**
   * Construct the container for the controller coefficients.
   *
   * @param K    Controller gain matrix.
   * @param Kff  Controller feedforward gain matrix.
   * @param Umin Minimum control input.
   * @param Umax Maximum control input.
   */
  public StateSpaceControllerCoeffs(Matrix<Inputs, States> K, Matrix<Inputs, States> Kff, Matrix<Inputs, N1> Umin,
      Matrix<Inputs, N1> Umax) {
    this.K = K;
    this.Kff = Kff;
    this.Umin = Umin;
    this.Umax = Umax;
  }

  public Matrix<Inputs, States> getK() {
    return K;
  }

  public Matrix<Inputs, States> getKff() {
    return Kff;
  }

  public Matrix<Inputs, N1> getUmin() {
    return Umin;
  }

  public double getUmin(int i) {
    return getUmin().get(i, 0);
  }

  public Matrix<Inputs, N1> getUmax() {
    return Umax;
  }

  public double getUmax(int i) {
    return getUmax().get(i, 0);
  }
}
