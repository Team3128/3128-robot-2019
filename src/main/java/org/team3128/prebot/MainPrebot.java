package org.team3128.prebot;

import edu.wpi.first.wpilibj.RobotBase;

public final class MainPrebot {
  private MainPrebot() {
  }

  public static void main(String... args) {
    RobotBase.startRobot(PrebotRobot::new);
  }
}
