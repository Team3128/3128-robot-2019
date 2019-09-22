/**
 * @author Adham Elarabawy, amg
 */
package org.team3128.common.hardware.motor;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Sends only new commands to the Talon to reduce CAN strain.
 */
public class LazyCANSparkMax extends CANSparkMax {

	private double prevValue = 0;
	private ControlType prevControlMode = ControlType.kVoltage;

	public LazyCANSparkMax(int deviceNumber, MotorType type) {
		super(deviceNumber, type);
		// enableVoltageCompensation(true);
		// configVoltageCompSaturation(12, 10);
	}

	@Override
	public void set(double outputValue) {
		// return;

		if (outputValue != prevValue) {
			super.set(outputValue);
			prevValue = outputValue;
		}

	}

	public double getSetpoint() {
		return prevValue;
	}
}
