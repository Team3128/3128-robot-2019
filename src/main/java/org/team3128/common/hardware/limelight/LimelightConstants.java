package org.team3128.common.hardware.limelight;

import org.team3128.common.util.units.Angle;

public class LimelightConstants {
    public static final String[] valueKeys = {"tx", "ty", "ts", "ta", "thor", "tvert", "tshort", "tlong", "tv"};
    public static final String[] valueKeysPnP = {"x", "y", "z", "pitch", "yaw", "roll"};

    public static final double SCREEN_WIDTH = 320;
    public static final double SCREEN_HEIGHT = 240;

    public static final double HORIZONTAL_FOV = 59.6 * Angle.DEGREES;
    public static final double VERTICAL_FOV = 45.7 * Angle.DEGREES;
}