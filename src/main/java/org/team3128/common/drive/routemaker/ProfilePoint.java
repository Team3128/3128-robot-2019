package org.team3128.common.drive.routemaker;

import com.ctre.phoenix.motion.TrajectoryPoint;

/**
 * Holds the left and right {@link TrajectoryPoint} for a specific target point
 * of the motion.
 * 
 * @author Ronak
 * 
 */
public class ProfilePoint {
    // public double x, y;
    public boolean last;
    // public double x_r, y_r;
    // public double x_l, y_l;
    public double leftDistance, rightDistance;
    public double leftSpeed, rightSpeed;

    public ProfilePoint(/*double x, double y,*/ boolean last, /* double x_l, double y_l, double x_r, double y_r,*/ double leftDistance, double rightDistance, double leftSpeed, double rightSpeed) {
        // this.x = x;
        // this.y = y;

        this.last = last;

        // this.x_l = x_l;
        // this.y_l = y_l;

        // this.x_r = x_r;
        // this.y_r = y_r;

        this.leftDistance = leftDistance;
        this.rightDistance = rightDistance;

        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightDistance;
    }
}