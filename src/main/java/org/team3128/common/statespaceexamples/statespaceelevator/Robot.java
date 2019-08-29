package org.team3128.common.statespaceexamples.statespaceelevator;

import edu.wpi.first.wpilibj.TimedRobot;

import org.team3128.common.statespaceexamples.statespaceelevator.subsystems.Elevator;

public class Robot extends TimedRobot {
  public static Elevator m_elevator;

  @Override
  public void robotInit() {
    m_elevator = new Elevator();
  }
}
