// package org.team3128.gromit.mechanisms;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;

// import org.team3128.common.hardware.misc.Piston;
// import org.team3128.common.utility.Log;

// import edu.wpi.first.wpilibj.command.Command;

// /**
// * Control system for the mechanism controlling mechanisms
// *
// * @author Chris, Jude, Tygan
// *
// */

// public class GroundIntake {
// public enum GroundIntakeState
// {
// DEPLOYED(0, true, "Deployed"),
// INTAKING(-1.0, false, "Intaking"),
// RETRACTED(0.0, true, "Retract");

// private double rollerPower;
// private boolean isClosed;
// private String name;

// private GroundIntakeState(double rollerPower, boolean isClosed, String name)
// {
// this.rollerPower = rollerPower;
// this.isClosed = isClosed;
// this.name = name;
// }

// public double getRollerPower() {
// return rollerPower;
// }
// public boolean getPistonPosition() {
// return isClosed;
// }

// public String getName() {
// return name;
// }
// }
// VictorSPX intakeMotors;
// private GroundIntakeState state;
// private Piston deployPistons;
// private double invertMultiplier;

// private static GroundIntake instance = null;
// public static GroundIntake getInstance() {
// if (instance != null) {
// return instance;
// }

// Log.fatal("GroundIntake", "Attempted to get instance before initializtion!
// Call initialize(...) first.");
// return null;
// }

// public static void initialize(VictorSPX intakeMotors, GroundIntakeState
// state, Piston deployPistons, boolean inverted) {
// instance = new GroundIntake(intakeMotors, state, deployPistons, inverted);
// }

// private GroundIntake(VictorSPX intakeMotors, GroundIntakeState state, Piston
// deployPistons, boolean inverted) {
// this.intakeMotors = intakeMotors;
// this.state = state;
// this.deployPistons = deployPistons;

// this.invertMultiplier = (inverted) ? -1 : 1;

// this.state = GroundIntakeState.RETRACTED;
// setState(GroundIntakeState.RETRACTED);
// }

// public void setState(GroundIntakeState newState) {
// if (state != newState) {
// if(newState.getPistonPosition()) {
// deployPistons.setPistonOn();
// }
// else {
// deployPistons.setPistonOff();
// }

// setIntakePower(newState.getRollerPower());

// state = newState;
// }
// }

// private void setIntakePower(double power) {
// intakeMotors.set(ControlMode.PercentOutput, invertMultiplier * power);
// }

// public class CmdSetGroundIntakeState extends Command {
// GroundIntakeState desiredState;

// public CmdSetGroundIntakeState(GroundIntakeState state) {
// super(0.1);

// desiredState = state;
// }

// @Override
// protected void initialize()
// {
// setState(desiredState);
// }

// @Override
// protected boolean isFinished()
// {
// return isTimedOut();
// }
// }
// }