package org.team3128.common.controller;

import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import org.team3128.common.util.trajectory.TrapezoidProfile;;

/**
 * Implements a PID control loop whose setpoint is constrained by a trapezoid
 * profile.
 */
@SuppressWarnings("PMD.TooManyMethods")
public class ProfiledPIDController extends SendableBase {
  private PIDController m_controller;
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.Constraints m_constraints;

  /**
   * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and
   * Kd.
   *
   * @param Kp          The proportional coefficient.
   * @param Ki          The integral coefficient.
   * @param Kd          The derivative coefficient.
   * @param constraints Velocity and acceleration constraints for goal.
   */
  @SuppressWarnings("ParameterName")
  public ProfiledPIDController(double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints) {
    this(Kp, Ki, Kd, constraints, 0.02);
  }

  /**
   * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and
   * Kd.
   *
   * @param Kp          The proportional coefficient.
   * @param Ki          The integral coefficient.
   * @param Kd          The derivative coefficient.
   * @param constraints Velocity and acceleration constraints for goal.
   * @param period      The period between controller updates in seconds. The
   *                    default is 0.02 seconds.
   */
  @SuppressWarnings("ParameterName")
  public ProfiledPIDController(double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints,
      double period) {
    m_controller = new PIDController(Kp, Ki, Kd, period);
    m_constraints = constraints;
  }

  /**
   * Sets the PID Controller gain parameters.
   *
   * <p>
   * Sets the proportional, integral, and differential coefficients.
   *
   * @param Kp Proportional coefficient
   * @param Ki Integral coefficient
   * @param Kd Differential coefficient
   */
  @SuppressWarnings("ParameterName")
  public void setPID(double Kp, double Ki, double Kd) {
    m_controller.setPID(Kp, Ki, Kd);
  }

  /**
   * Sets the proportional coefficient of the PID controller gain.
   *
   * @param Kp proportional coefficient
   */
  @SuppressWarnings("ParameterName")
  public void setP(double Kp) {
    m_controller.setP(Kp);
  }

  /**
   * Sets the integral coefficient of the PID controller gain.
   *
   * @param Ki integral coefficient
   */
  @SuppressWarnings("ParameterName")
  public void setI(double Ki) {
    m_controller.setI(Ki);
  }

  /**
   * Sets the differential coefficient of the PID controller gain.
   *
   * @param Kd differential coefficient
   */
  @SuppressWarnings("ParameterName")
  public void setD(double Kd) {
    m_controller.setD(Kd);
  }

  /**
   * Gets the proportional coefficient.
   *
   * @return proportional coefficient
   */
  public double getP() {
    return m_controller.getP();
  }

  /**
   * Gets the integral coefficient.
   *
   * @return integral coefficient
   */
  public double getI() {
    return m_controller.getI();
  }

  /**
   * Gets the differential coefficient.
   *
   * @return differential coefficient
   */
  public double getD() {
    return m_controller.getD();
  }

  /**
   * Gets the period of this controller.
   *
   * @return The period of the controller.
   */
  public double getPeriod() {
    return m_controller.getPeriod();
  }

  /**
   * Sets the goal for the ProfiledPIDController.
   *
   * @param goal The desired unprofiled setpoint.
   */
  public void setGoal(double goal) {
    m_goal = new TrapezoidProfile.State(goal, 0);
  }

  /**
   * Gets the goal for the ProfiledPIDController.
   */
  public double getGoal() {
    return m_goal.position;
  }

  /**
   * Returns true if the error is within tolerance of the setpoint.
   *
   * <p>
   * This will return false until at least one input value has been computed.
   *
   * @param positionTolerance The maximum allowable position error.
   */
  public boolean atGoal(double positionTolerance) {
    return atGoal(positionTolerance, Double.POSITIVE_INFINITY, PIDController.Tolerance.kAbsolute);
  }

  /**
   * Returns true if the error is within tolerance of the setpoint.
   *
   * <p>
   * This will return false until at least one input value has been computed.
   *
   * @param positionTolerance The maximum allowable position error.
   * @param velocityTolerance The maximum allowable velocity error.
   */
  public boolean atGoal(double positionTolerance, double velocityTolerance) {
    return atGoal(positionTolerance, velocityTolerance, PIDController.Tolerance.kAbsolute);
  }

  /**
   * Returns true if the error is within tolerance of the setpoint.
   *
   * <p>
   * This will return false until at least one input value has been computed.
   *
   * @param positionTolerance The maximum allowable position error.
   * @param velocityTolerance The maximum allowable velocity error.
   * @param toleranceType     The type of tolerance specified.
   */
  public boolean atGoal(double positionTolerance, double velocityTolerance, PIDController.Tolerance toleranceType) {
    return atSetpoint(positionTolerance, velocityTolerance, toleranceType) && m_goal.equals(m_setpoint);
  }

  /**
   * Returns true if the error is within the tolerance of the error.
   *
   * <p>
   * This will return false until at least one input value has been computed.
   */
  public boolean atGoal() {
    return atSetpoint() && m_goal.equals(m_setpoint);
  }

  /**
   * Set velocity and acceleration constraints for goal.
   *
   * @param constraints Velocity and acceleration constraints for goal.
   */
  public void setConstraints(TrapezoidProfile.Constraints constraints) {
    m_constraints = constraints;
  }

  /**
   * Returns the current setpoint of the ProfiledPIDController.
   *
   * @return The current setpoint.
   */
  public double getSetpoint() {
    return m_controller.getSetpoint();
  }

  /**
   * Returns true if the error is within tolerance of the setpoint.
   *
   * <p>
   * This will return false until at least one input value has been computed.
   *
   * @param positionTolerance The maximum allowable position error.
   */
  public boolean atSetpoint(double positionTolerance) {
    return atSetpoint(positionTolerance, Double.POSITIVE_INFINITY, PIDController.Tolerance.kAbsolute);
  }

  /**
   * Returns true if the error is within tolerance of the setpoint.
   *
   * <p>
   * This will return false until at least one input value has been computed.
   *
   * @param positionTolerance The maximum allowable position error.
   * @param velocityTolerance The maximum allowable velocity error.
   */
  public boolean atSetpoint(double positionTolerance, double velocityTolerance) {
    return atSetpoint(positionTolerance, velocityTolerance, PIDController.Tolerance.kAbsolute);
  }

  /**
   * Returns true if the error is within tolerance of the setpoint.
   *
   * <p>
   * This will return false until at least one input value has been computed.
   *
   * @param positionTolerance The maximum allowable position error.
   * @param velocityTolerance The maximum allowable velocity error.
   * @param toleranceType     The type of tolerance specified.
   */
  public boolean atSetpoint(double positionTolerance, double velocityTolerance, PIDController.Tolerance toleranceType) {
    return m_controller.atSetpoint(positionTolerance, velocityTolerance, toleranceType);
  }

  /**
   * Returns true if the error is within the tolerance of the error.
   *
   * <p>
   * This will return false until at least one input value has been computed.
   */
  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  /**
   * Sets the minimum and maximum values expected from the input.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  public void setInputRange(double minimumInput, double maximumInput) {
    m_controller.setInputRange(minimumInput, maximumInput);
  }

  /**
   * Enables continuous input.
   *
   * <p>
   * Rather then using the max and min input range as constraints, it considers
   * them to be the same point and automatically calculates the shortest route to
   * the setpoint.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  public void enableContinuousInput(double minimumInput, double maximumInput) {
    m_controller.enableContinuousInput(minimumInput, maximumInput);
  }

  /**
   * Disables continuous input.
   */
  public void disableContinuousInput() {
    m_controller.disableContinuousInput();
  }

  /**
   * Sets the minimum and maximum values to write.
   *
   * @param minimumOutput the minimum value to write to the output
   * @param maximumOutput the maximum value to write to the output
   */
  public void setOutputRange(double minimumOutput, double maximumOutput) {
    m_controller.setOutputRange(minimumOutput, maximumOutput);
  }

  /**
   * Sets the absolute error which is considered tolerable for use with
   * atSetpoint().
   *
   * @param positionTolerance Position error which is tolerable.
   */
  public void setAbsoluteTolerance(double positionTolerance) {
    setAbsoluteTolerance(positionTolerance, Double.POSITIVE_INFINITY);
  }

  /**
   * Sets the absolute error which is considered tolerable for use with
   * atSetpoint().
   *
   * @param positionTolerance Position error which is tolerable.
   * @param velocityTolerance Velocity error which is tolerable.
   */
  public void setAbsoluteTolerance(double positionTolerance, double velocityTolerance) {
    m_controller.setAbsoluteTolerance(positionTolerance, velocityTolerance);
  }

  /**
   * Sets the percent error which is considered tolerable for use with
   * atSetpoint().
   *
   * @param positionTolerance Position error which is tolerable.
   */
  public void setPercentTolerance(double positionTolerance) {
    m_controller.setPercentTolerance(positionTolerance, Double.POSITIVE_INFINITY);
  }

  /**
   * Sets the percent error which is considered tolerable for use with
   * atSetpoint().
   *
   * @param positionTolerance Position error which is tolerable.
   * @param velocityTolerance Velocity error which is tolerable.
   */
  public void setPercentTolerance(double positionTolerance, double velocityTolerance) {
    m_controller.setPercentTolerance(positionTolerance, velocityTolerance);
  }

  /**
   * Returns the difference between the setpoint and the measurement.
   *
   * @return The error.
   */
  public double getPositionError() {
    return m_controller.getPositionError();
  }

  /**
   * Returns the change in error per second.
   */
  public double getVelocityError() {
    return m_controller.getVelocityError();
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   */
  public double calculate(double measurement) {
    var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
    m_setpoint = profile.calculate(getPeriod());
    return m_controller.calculate(measurement, m_setpoint.position);
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @param goal        The new goal of the controller.
   */
  public double calculate(double measurement, double goal) {
    setGoal(goal);
    return calculate(measurement);
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @param goal        The new goal of the controller.
   * @param constraints Velocity and acceleration constraints for goal.
   */
  public double calculate(double measurement, double goal, TrapezoidProfile.Constraints constraints) {
    setConstraints(constraints);
    return calculate(measurement, goal);
  }

  /**
   * Reset the previous error, the integral term, and disable the controller.
   */
  public void reset() {
    m_controller.reset();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ProfiledPIDController");
    builder.addDoubleProperty("p", this::getP, this::setP);
    builder.addDoubleProperty("i", this::getI, this::setI);
    builder.addDoubleProperty("d", this::getD, this::setD);
    builder.addDoubleProperty("goal", this::getGoal, this::setGoal);
  }
}
