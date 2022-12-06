// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Autonomous extends SubsystemBase {
  public Tracking Limelight;
  public SwerveDrive SwerveDrive;
  public Autonomous() {
    Limelight = new Tracking();
  }

  @Override
  public void periodic() {

  }
}
