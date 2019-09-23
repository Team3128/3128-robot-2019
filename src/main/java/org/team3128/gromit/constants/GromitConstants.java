package org.team3128.gromit.constants;

import org.team3128.common.generics.RobotConstants;
import org.team3128.common.utility.units.Length;
import org.team3128.common.utility.units.Angle;

public class GromitConstants extends RobotConstants {

    // PNUEMATICS CONSTANTS:
    public static final int GEARSHIFT_SOL_A = 3;
    public static final int GEARSHIFT_SOL_B = 4;

    public static final int CLIMB_SOL_A = 1;
    public static final int CLIMB_SOL_B = 6;

    public static final int HATCH_INTAKE_SOL_A = 0;
    public static final int HATCH_INTAKE_SOL_B = 7;

    public static final int PLACEHOLDER_SOL_A = 2;
    public static final int PLACEHOLDER_SOL_B = 5;

    // IO CONSTANTS:
    public static final int LIFT_LIMIT_SWITCH = 8;
    public static final int FOURBAR_LIMIT_SWITCH = 0;

    // MECHANISM CONSTANTS:

    // ---- DRIVE
    public static final double WHEELBASE = 37 * Length.in;
    public static final int DRIVE_MAX_SPEED = 5800;
    public static final double WHEEL_CIRCUMFERENCE = 12.01 * Length.in;
    public static final double LEFT_SPEED_SCALAR = 1.00;
    public static final double RIGHT_SPEED_SCALAR = 1.00;

    public static final double SHIFT_UP_SPEED = 100000;
    public static final double SHIFT_DOWN_SPEED = -1;

    // ---- AUTONOMOUS DRIVE
    public static final double TRACK_RADIUS = -12 * Length.in;
    public static final double WHEEL_DIAMETER = 6.0 * Length.in;
    public static final double MIN_TURNING_RADIUS = 40 * Length.in;
    public static final double MIN_PATH_SPEED = 20 * Length.in;
    public static final double MAX_PATH_SPEED = 120 * Length.in;
    public static final double MIN_LOOKAHEAD_DISTANCE = 14 * Length.in;
    public static final double MAX_LOOKAHEAD_DISTANCE = 30 * Length.in;

    // ---- FOURBAR
    public static final double FOURBAR_RATIO = 4600 / (180 * Angle.DEGREES);
    public static final double FOURBAR_SWITCH_POSITION = +96 * Angle.DEGREES;
    public static final int FOURBAR_MAX_VELOCITY = 100;

    // ---- LIFT
    public static final int LIFT_SWITCH_POSITION = 40;
    public static final int LIFT_MAX_VELOCITY = 4200;

    // ---- LIMELIGHT
    public static final double BOTTOM_LIMELIGHT_HEIGHT = 6.15 * Length.in;
    public static final double BOTTOM_LIMELIGHT_ANGLE = 38.0 * Angle.DEGREES;

    public static final double TOP_LIMELIGHT_HEIGHT = 43 * Length.in;
    public static final double TOP_LIMELIGHT_ANGLE = -12.0 * Angle.DEGREES;

}