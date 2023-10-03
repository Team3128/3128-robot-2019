package org.team3128.common.statespaceexamples.periodvariantelevator.subsystems;

import org.team3128.common.controller.PeriodVariantLoop;
import org.team3128.common.util.math.Matrix;
import org.team3128.common.util.math.MatrixUtils;
import org.team3128.common.util.math.Nat;
import org.team3128.common.util.math.numbers.N1;
import org.team3128.common.util.math.numbers.N2;

@SuppressWarnings("MemberName")
public class ElevatorController {
  // State tolerances in meters and meters/sec respectively.
  public static final double kPositionTolerance = 0.05;
  public static final double kVelocityTolerance = 2.0;

  // The current sensor measurement.
  private final Matrix<N1, N1> m_Y;

  // The control loop.
  private final PeriodVariantLoop<N2, N1, N1> m_loop = ElevatorCoeffs.makeElevatorLoop();

  private boolean m_atReferences;

  public ElevatorController() {
    m_Y = MatrixUtils.zeros(Nat.N1());
  }

  public void enable() {
    m_loop.enable();
  }

  public void disable() {
    m_loop.disable();
  }

  /**
   * Sets the references.
   *
   * @param position Position of the carriage in meters.
   * @param velocity Velocity of the carriage in meters per second.
   */
  public void setReferences(double position, double velocity) {
    Matrix<N2, N1> nextR = MatrixUtils.vec(Nat.N2()).fill(position, velocity);
    m_loop.setNextR(nextR);
  }

  public boolean atReferences() {
    return m_atReferences;
  }

  /**
   * Sets the current encoder measurement.
   *
   * @param measuredPosition Position of the carriage in meters.
   */
  public void setMeasuredPosition(double measuredPosition) {
    m_Y.set(0, 0, measuredPosition);
  }

  public double getControllerVoltage() {
    return m_loop.getU(0);
  }

  public double getEstimatedPosition() {
    return m_loop.getXhat(0);
  }

  public double getEstimatedVelocity() {
    return m_loop.getXhat(1);
  }

  public double getPositionError() {
    return m_loop.getError().get(0, 0);
  }

  public double getVelocityError() {
    return m_loop.getError().get(1, 0);
  }

  /**
   * Executes the control loop for a cycle.
   *
   * @param dt Measured time since the last controller update.
   */
  public void update(double dt) {
    m_loop.correct(m_Y);

    var error = m_loop.getError();
    m_atReferences = Math.abs(error.get(0, 0)) < kPositionTolerance && Math.abs(error.get(1, 0)) < kVelocityTolerance;
    m_loop.predict(dt);
  }

  /**
   * Resets any internal state.
   */
  public void reset() {
    m_loop.reset();
  }
}
