package org.team3128.common.hardware.limelight;

import org.team3128.common.util.units.Angle;

public class LimelightConstants {
    public static final String[] valueKeys = {"tx", "ty", "ts", "ta", "thor", "tvert", "tshort", "tlong"};
    public static final String[] valueKeysPnP = {"x", "y", "z", "pitch", "yaw", "roll"};

    public static final double screenWidth = 320;
    public static final double screenHeight = 240;
    public static final double horizFOV = 59.6 * Angle.DEGREES;
    public static final double vertFOV = 45.7 * Angle.DEGREES;
}