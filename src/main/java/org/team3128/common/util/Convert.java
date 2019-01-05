package org.team3128.common.util;

public class Convert {
    /**
     * Converts velocity from units of native units/100ms to cm/s
     * 
     * @param ctre - the speed in native units/100ms
     * @param circumfrence - the circumfrence of the drive wheels in centimeters
     * 
     * @return the velocity in cm per second
     */
    public static double velocityCTREtoCMS(double ctre, double circumfrence) {
        return (10 * ctre / 4096) * circumfrence;
    }

    /**
     * Converts velocity from units of cm/s to native units/100ms
     * 
     * @param cms - the speed in cm/s
     * @param circumfrence - the circumfrence of the drive wheels in centimeters
     * 
     * @return the velocity in native units per 100ms
     */
    public static double velocityCMStoCTRE(double cms, double circumfrence) {
        return (cms / circumfrence) * 4096 / 10;
    }

    public static double lengthCMtoCTRE(double cms, double circumfrence) {
        return 4096 * cms / circumfrence;
    }

    public static double lengthCTREtoCM(double ctre, double circumfrence) {
        return circumfrence * ctre / 4096;
    }
}