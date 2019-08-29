package org.team3128.common.statespaceexamples.statespaceelevator;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {
  }

  public static void main(String[] args) {
    RobotBase.startRobot(Robot::new);
  }
}
