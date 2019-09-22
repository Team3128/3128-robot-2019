package org.team3128.common.hardware.gyroscope;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/**
 * Class that implements Team 3128's {@link Gyro} interface in order to allow
 * for fetching and setting of commonly needed values for the Kauai Labs NavX
 * IMU.
 */
public class NavX implements Gyro {
    /**
     * The actual, Kauai Labs-supplied, fully-functional Java object version of the
     * NavX IMU.
     */
    private AHRS ahrs;

    public NavX() {
        ahrs = new AHRS(SPI.Port.kMXP);
    }

    @Override
    public double getAngle() {
        return -ahrs.getAngle();
    }

    @Override
    public double getRate() {
        return -Math.toDegrees(ahrs.getRate());
    }

    @Override
    public double getPitch() {
        return ahrs.getPitch();
    }

    @Override
    public double getRoll() {
        return ahrs.getRoll();
    }

    @Override
    public void reset() {
        ahrs.reset();
    }

    @Override
    public void setAngle(double angle) {
        ahrs.reset();
        ahrs.setAngleAdjustment(angle);
    }
}