package org.team3128.common.statespaceexamples.statespacedifferentialdrive;

import edu.wpi.first.wpilibj.TimedRobot;

import org.team3128.common.statespaceexamples.statespacedifferentialdrive.subsystems.Drivetrain;

public class Robot extends TimedRobot {
  public static Drivetrain m_drivetrain;

  @Override
  public void robotInit() {
    m_drivetrain = new Drivetrain();
  }
}
