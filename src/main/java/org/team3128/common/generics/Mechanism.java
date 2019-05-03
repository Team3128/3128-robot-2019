package org.team3128.common.generics;

import org.team3128.common.util.Log;

import edu.wpi.first.wpilibj.Notifier;

/**
 * The Mechanism abstract class serves as a base framework for any mechanism on the robot
 * (elevator, shooter, arm, turret, drivetrain, etc.)
 * <p>
 * Mechanisms should follow a Singelton class pattern (since only one instance of any given
 * mechanism will actually exist on the robot).
 * <p>
 * Mechanisms may also choose to implement the control loop model, in which acutators (namely,
 * feedback driven motors) are controlled indirectly through the input of set points. The logic
 * executed within each cycle of the control loop is expected to monitor the current state of the
 * mechanism and provide outputs to drive the mechanism towards the desired state.
 */
public abstract class Mechanism implements Loggable {
    private Notifier controlNotifier;
    private boolean enabled = false;

    /**
     * Start the looping of the internal control loop with a loop period of 10ms.
     */
    public void startControlNotifier() {
        startControlNotifier(10);
    }

    /**
     * Start the looping of the internal control loop with the specified loop period.
     */
    public void startControlNotifier(int periodMs) {
        Log.debug(this, "Starting Control Notifier...");

        controlNotifier = new Notifier(() -> {
            if (enabled) {
                controlLoop();
            }
        });
        controlNotifier.startPeriodic(periodMs / 1000.0);
    }

    /**
     * The method to run every cycle of the optional internal control loop.
     */
    protected void controlLoop() {

    }
    
    /**
     * Places the mechanism in a state in which the mechanism drives itself to some
     * state with known sensor values (indicated by limit switch activation or the arrival
     * at a hard stop), in which all sensor values are reset upon arrival.
     */
    public abstract void zero();

    /**
     * Called whenever the robot enters the enabled period. Should be used to do any
     * recallibration that needs to be done to account for anything that may occur
     * while the robot is disabled.
     */
    public abstract void enable();

    /**
     * Called whenever the robot enters the disabled period, like at the end of
     * the match or when the robot is emergenct disabled when something goes wrong
     * in testing. Should set all motor powers to zero and return all pistons to the
     * resting/inactive state.
     */
    public abstract void disable();

    /**
     * Called by NarwhalRobot when the robot switches Modes.
     * 
     * @param enabled
     */
    public void setEnabled(boolean enabled) {
        if (enabled && !this.enabled) {
            enable();
        }
        else if (!enabled && this.enabled) {
            disable();
        }

        this.enabled = enabled;
    }
}