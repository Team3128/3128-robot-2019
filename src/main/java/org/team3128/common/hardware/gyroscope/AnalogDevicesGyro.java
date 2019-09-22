package org.team3128.common.hardware.gyroscope;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

/**
 * Class that implements Team 3128's {@link Gyro} interface in order to allow
 * for fetching and setting of commonly needed values for the Analog Devices
 * ADXRS450 Gyro that comes in the KoP.
 */
public class AnalogDevicesGyro implements Gyro {
    /**
     * The actual, WPIlib-supplied, fully-functional Java object version of the
     * gyro.
     */
    public ADXRS450_Gyro gyro;

    private double offset;

    public AnalogDevicesGyro() {
        gyro = new ADXRS450_Gyro();

        offset = 0;
    }

    @Override
    public double getAngle() {
        return offset - gyro.getAngle();
    }

    @Override
    public double getPitch() {
        return -10000;
    }

    @Override
    public double getRoll() {
        return -10000;
    }

    @Override
    public double getRate() {
        return -1 * gyro.getRate();
    }

    @Override
    public void reset() {
        gyro.reset();
        offset = 0;
    }

    @Override
    public void setAngle(double angle) {
        gyro.reset();

        offset = angle;
    }

    public void recalibrate() {
        gyro.calibrate();
    }
}