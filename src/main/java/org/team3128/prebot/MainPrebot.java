package org.team3128.prebot;

import org.team3128.common.NarwhalRobot;

import edu.wpi.first.wpilibj.RobotBase;

public class MainPrebot extends NarwhalRobot {
	@Override
	protected void constructHardware() {
        
    }
    
    @Override
    protected void constructAutoPrograms() {

    }

	@Override
	protected void setupListeners() {
		
	}
    
    @Override
    protected void updateDashboard() {
        
    }

    public static void main(String... args) {
        RobotBase.startRobot(MainPrebot::new);
    }
}
