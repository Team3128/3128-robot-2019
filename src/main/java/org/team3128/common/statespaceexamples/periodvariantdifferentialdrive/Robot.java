package org.team3128.common.statespaceexamples.periodvariantdifferentialdrive;

import edu.wpi.first.wpilibj.TimedRobot;

import org.team3128.common.statespaceexamples.periodvariantdifferentialdrive.subsystems.Drivetrain;

public class Robot extends TimedRobot {
  public static Drivetrain m_drivetrain;

  @Override
  public void robotInit() {
    m_drivetrain = new Drivetrain();
  }
}
