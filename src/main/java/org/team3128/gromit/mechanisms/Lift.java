package org.team3128.gromit.mechanisms;

import org.team3128.common.generics.Mechanism;
import org.team3128.common.util.Constants;
import org.team3128.common.util.Log;
import org.team3128.common.util.units.Length;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Control system for the two-stage, continuous elevator-style lift mechanism. The
 * {@link FourBar} is mounted to the first stage of the list.
 * 
 * @author Chris, Jude, Tygan
 * 
 */

public class Lift extends Mechanism {
	@Override
	public String getTag() {
		return "Lift";
	}

	public enum LiftHeightTarget {
		BASE(0 * Length.in),
		
		INTAKE_FLOOR_CARGO(8 * Length.in),
		VISION(35 * Length.in),

		SHIP_CARGO(30 * Length.in),

		LOW_CARGO(16 * Length.in),
		MID_CARGO(49 * Length.in),
		TOP_CARGO(80 * Length.in),

		HATCH_INTAKE(21.5 * Length.in),
		HATCH_PULL_UP(28.5 * Length.in),
		
        LOW_HATCH(26.5 * Length.in),
        MID_HATCH(33.5 * Length.in),
		TOP_HATCH(67.5 * Length.in); // 69.5

		public double targetHeight;

		private LiftHeightTarget(double height) {
			this.targetHeight = height;
		}
	}

	public enum LiftControlMode {
		ZEROING(-1, "Zeroing"),
		PERCENT(-1, "Percent Output"),
		POSITION_UP(0, "Position (Up)"),
		POSITION_DOWN(1, "Position (Down)");

		private int pidSlot;
		private String name;

		private LiftControlMode(int pidSlot, String name) {
			this.pidSlot = pidSlot;
			this.name = name;
		}

		public int getPIDSlot() {
			return pidSlot;
		}

		public ControlMode getTalonControlMode() {
			return (pidSlot == -1) ? ControlMode.PercentOutput : ControlMode.MotionMagic;
		}

		public String getName() {
			return name;
		}

	}

	public void setControlMode(LiftControlMode controlMode) {
		if (this.controlMode != controlMode) {
			this.controlMode = controlMode;

			Log.debug(this, "Setting control mode to " + controlMode.getName() + ", with PID slot " + controlMode.getPIDSlot());
			liftMotor.selectProfileSlot(controlMode.getPIDSlot(), 0);
		}
	}

	/**
	 * The amount of encoder rotation in native units that results in lift movement of 1 centimeter.
	 * 
	 * MAX ENCODER POSITION = 51,745 native units
	 * MAX HEIGHT = 78.5 inches
	 */
	public final double MAX_HEIGHT = 78 * Length.in;
	public final double ratio = 51745 / (78.8 * Length.in);

	// Physical Components
	private TalonSRX liftMotor;
	private DigitalInput limitSwitch;

	public int limitSwitchLocation, liftMaxVelocity;

	// Control Notifier
	public LiftControlMode controlMode;

	public final double BRAKE_POWER = 0.15;
	public final double CONTROL_BUFFER = 2 * Length.in;

	public boolean canLower = false;
	public boolean canRaise = true;

	private double desiredTarget = 0;
	private double currentTarget, previousTarget;
	public double error;

	public boolean override = false;

	private int zeroVelocityCount = 0;
	private boolean previousSwitchState = false;

	private static Lift instance = null;
	public static Lift getInstance() {
		if (instance != null) {
			return instance;
		}

		Log.fatal(instance, "Attempted to get instance before initializtion! Call initialize(...) first.");
		return null;
	}

	public static void initialize(TalonSRX liftMotor, DigitalInput limitSwitch, int limitSwitchLocation, int liftMaxVelocity) {
		instance = new Lift(liftMotor, limitSwitch, limitSwitchLocation, liftMaxVelocity);
	}

	private Lift(TalonSRX liftMotor, DigitalInput limitSwitch, int limitSwitchLocation, int liftMaxVelocity) {
		this.liftMotor = liftMotor;

		this.limitSwitch = limitSwitch;
		this.liftMaxVelocity = liftMaxVelocity;

		this.limitSwitchLocation = limitSwitchLocation;

		setControlMode(LiftControlMode.PERCENT);

		liftMotor.configMotionCruiseVelocity((int) (0.9 * liftMaxVelocity), Constants.CAN_TIMEOUT);
		liftMotor.configMotionAcceleration((int) (1.5 * liftMaxVelocity), Constants.CAN_TIMEOUT);

		liftMotor.configOpenloopRamp(0.2, Constants.CAN_TIMEOUT);
	}

	@Override
	protected void controlLoop() {
		if (getLimitSwitch() != previousSwitchState) {
			liftMotor.setSelectedSensorPosition(limitSwitchLocation, 0, Constants.CAN_TIMEOUT);

			previousSwitchState = getLimitSwitch();
		}

		currentTarget = 0;

		switch (controlMode) {
			case ZEROING:
				currentTarget = -0.2;

				if (Math.abs(liftMotor.getSelectedSensorVelocity()) < 10) {
					zeroVelocityCount += 1;
				}
				else {
					zeroVelocityCount = 0;
				}
	
				if (zeroVelocityCount > 5 || getLimitSwitch()) {
					powerControl(0);
	
					Log.info(this, "Zeroing sequence hit soft/hard stop. Braking now...");
	
					zeroVelocityCount = 0;
				}

				break;
			
			case PERCENT:
				canRaise = getCurrentHeight() < MAX_HEIGHT - CONTROL_BUFFER;
				canLower = getCurrentHeight() > 0;

				if (override) {
					currentTarget = desiredTarget;
				}
				else {
					if (desiredTarget > 0 && canRaise) {
						currentTarget = desiredTarget;
					}
					else if (desiredTarget < 0 && canLower) {
						currentTarget = 0.7 * desiredTarget;
					}

					if ((Math.abs(currentTarget) < 0.05 && getCurrentHeight() >= 3 * Length.in)) {
						currentTarget = BRAKE_POWER;
					}
				}

				break;

			case POSITION_UP:
			case POSITION_DOWN:
				currentTarget = desiredTarget;
				break;
		}

		if (Math.abs(currentTarget - previousTarget) > 0.0001) {
			liftMotor.set(controlMode.getTalonControlMode(), currentTarget);

			previousTarget = currentTarget;
		}
	}

	@Override
	public void zero() {
		setControlMode(LiftControlMode.ZEROING);
	}

	@Override
	public void enable() {
		override = false;
		powerControl(0);
	}

	@Override
	public void disable() {
		override = true;
		powerControl(0);

		liftMotor.set(ControlMode.PercentOutput, 0);
	}

	public double getCurrentHeight() {
		return liftMotor.getSelectedSensorPosition(0) / ratio;
	}

	public boolean getLimitSwitch() {
		return !limitSwitch.get();
	}

	public void heightControl(LiftHeightTarget heightTarget) {
		if (Math.abs(getCurrentHeight() - heightTarget.targetHeight) > 1 * Length.in) {
			if (getCurrentHeight() < heightTarget.targetHeight) {
				setControlMode(LiftControlMode.POSITION_UP);
			}
			else {
				setControlMode(LiftControlMode.POSITION_DOWN);
			}
	
			Log.info(this, "Setting height to " + heightTarget.targetHeight + " cm.");

			desiredTarget = heightTarget.targetHeight;
		}
	}

	public void powerControl(double joystick) {
		setControlMode(LiftControlMode.PERCENT);

		desiredTarget = joystick;
	}

	public class CmdHeightControl extends Command {
		private final static double MOVEMENT_ERROR_THRESHOLD = 6 * Length.in;
		private final static double ERROR_PLATEAU_THRESHOLD = 0.01 * Length.in;

		private final static int ERROR_PLATEAU_COUNT = 10;
		private int plateauCount = 0;

		private LiftHeightTarget heightState;

		public CmdHeightControl(LiftHeightTarget heightState) {
			super(3);
			this.heightState = heightState;
		}

		@Override
		protected void initialize() {
			heightControl(heightState);
			Log.debug("CmdHeightControl", "Setting target state to " + heightState.name() + " with target height " + (getCurrentHeight() / Length.in) + " in.");
		}

		@Override
		protected void end() {
			powerControl(0);
			Log.debug("CmdHeightControl", "Lift at height of " + (getCurrentHeight() / Length.in) + " inches.");
		}

		@Override
		protected void interrupted() {
			Log.debug("CmdHeightControl", "Command interrupted.");
			end();
		}

		@Override
		protected boolean isFinished() {
			error = getCurrentHeight() - heightState.targetHeight;

			Log.debug("CmdHeightControl", "Error: " + (error / Length.in) + " inches");

			if (Math.abs(error) > MOVEMENT_ERROR_THRESHOLD) return false;

			if (Math.abs(error) < ERROR_PLATEAU_THRESHOLD) {
				plateauCount += 1;
			}
			else {
				plateauCount = 0;
			}

			if (plateauCount > ERROR_PLATEAU_COUNT) return true;

			return isTimedOut();
		}
	}
}