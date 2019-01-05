package org.team3128.common.drive.routemaker;

/**
 * Represents the series of quintic spline segments that connects a series of waypoints.
 * 
 * @author Ronak
 * 
 */
public class Trajectory {
    private Segment[] segments;

    public Trajectory(Segment... segments) {
        this.segments = segments;
    }

    public Segment[] getSegments() {
        return segments;
    }

    public Waypoint getStart() {
        return segments[0].getStart();
    }

    public String toString() {
        String result = "";

        for (Segment segment:segments) {
            result += segment + "\n";
        }

        return result;
    }
}