package org.team3128.athos.subsystems;

import org.team3128.common.utility.units.Length;
import org.team3128.common.utility.units.Angle;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.generics.RobotConstants;

// the superclass is purely for semantic purposes
public class Constants extends RobotConstants {

        // MECHANISM CONSTANTS:

        public static final double inchesToMeters = 0.0254;
        public static final double kDriveInchesPerSecPerRPM = 2 * Math.PI / 60d * Constants.WHEEL_DIAMETER / 2d
                        * Constants.WHEEL_ROTATIONS_FOR_ONE_ENCODER_ROTATION; // a fairly basic relationship between
                                                                              // tangential and
        // rotational speed: rpm * 1min/60seconds * CIRCUM *
        // (relation between encoder rotations and wheel
        // rotations) = in/s

        // ---- DRIVE
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless; // indicates that we are using brushless motors
        public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;

        public static final double ENCODER_ROTATIONS_FOR_ONE_WHEEL_ROTATION = 75 / 9; // basically your gearing. Ask
                                                                                      // Mech
                                                                                      // for gear teeth number to gear
                                                                                      // teeth number ratio: 8.3333333

        public static final double WHEEL_ROTATIONS_FOR_ONE_ENCODER_ROTATION = 1
                        / Constants.ENCODER_ROTATIONS_FOR_ONE_WHEEL_ROTATION;

        public static final int RIGHT_DRIVE_FRONT_ID = 1;
        public static final int RIGHT_DRIVE_MIDDLE_ID = 2;
        public static final int RIGHT_DRIVE_BACK_ID = 0;

        public static final int LEFT_DRIVE_FRONT_ID = 5;
        public static final int LEFT_DRIVE_MIDDLE_ID = 4;
        public static final int LEFT_DRIVE_BACK_ID = 3;

        // public static final double WHEELBASE = 32.3; // Might be deprecated. I have to find out how to implement arc
                                                     // turns
                                                     // first. (Adham)
        public static final int DRIVE_HIGH_SPEED = 140; // Empirical Max Linear Speed: 146.57 in/s
        // public static final double WHEEL_CIRCUMFERENCE = 13.21; // Might be
        // deprecated. I am thinking of switching to
        // using
        // wheel diameter, as that is significantly easier to
        // measure.
        public static final double WHEEL_DIAMETER = 3.55; // effective wheel diameter (measure first then tune this
                                                          // number until distances are accurate)

        public static final double LEFT_SPEEDSCALAR = 1.0; // purely for TELEOP drive (to make sure that when the drive
                                                           // pushes the joystick forward, both sides of the drivetrain
                                                           // are
                                                           // going ROUGHLY the same speed)
        public static final double RIGHT_SPEEDSCALAR = 1.0;// purely for TELEOP drive (to make sure that when the drive
                                                           // pushes the joystick forward, both sides of the drivetrain
                                                           // are
                                                           // going ROUGHLY the same speed)

        public static final double DRIVE_ACCEL_LIMIT = 120; // Ballpark estimates from mech (Be conservative unless you
                                                            // really need the quick auto paths)
        public static final double DRIVE_JERK_LIMIT = 2000; // Ballpark estimates (Be conservative)

        public static double K_AUTO_RIGHT_P = 0.00007; // 0.00065
        public static double K_AUTO_RIGHT_D = 0.000;
        public static double K_AUTO_RIGHT_F = 1 / 145.9150145782 * kDriveInchesPerSecPerRPM; // 1/ (consistent max vel
                                                                                             // of this side of
                                                                                             // drivetrain in/s) *
                                                                                             // conversion to rpm
        public static double K_AUTO_LEFT_P = 0.00007;
        public static double K_AUTO_LEFT_D = 0.000; // 0.0001
        public static double K_AUTO_LEFT_F = 1 / 140.8705712261 * kDriveInchesPerSecPerRPM; // 1/ (consistent max vel of
                                                                                            // this side of drivetrain
                                                                                            // in/s) * conversion to rpm

        public static final double K_HOLD_P = 4;

        // ---- AUTONOMOUS DRIVE
        public static final double TRACK_RADIUS = 28;
        public static final double MIN_TURNING_RADIUS = 40;
        public static final double MIN_PATH_SPEED = 20;
        public static final double MAX_PATH_SPEED = 120;
        public static final double MIN_LOOKAHEAD_DISTANCE = 14;
        public static final double MAX_LOOKAHEAD_DISTANCE = 30;

        public static final double MAX_TURN_ERROR = 2;
        public static final double MAX_PID_STOP_SPEED = 8;

        // ---- FOURBAR
        public static final double FOURBAR_RATIO = 4600 / (180 * Angle.DEGREES);
        public static final double FOURBAR_SWITCH_POSITION = +96 * Angle.DEGREES;
        public static final int FOURBAR_MAX_VELOCITY = 100;

        // ---- LIFT
        public static final int LIFT_SWITCH_POSITION = 40;
        public static final int LIFT_MAX_VELOCITY = 4200;

        // ---- LIMELIGHT
        public static final double BOTTOM_LIMELIGHT_HEIGHT = 6.15 * Length.in;
        public static final double BOTTOM_LIMELIGHT_ANGLE = 26.0 * Angle.DEGREES;
        public static final double BOTTOM_LIMELIGHT_DISTANCE_FROM_FRONT = 0 * Length.in;

}