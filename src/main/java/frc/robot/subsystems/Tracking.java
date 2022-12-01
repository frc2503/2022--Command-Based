// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Tracking extends SubsystemBase {
  public NetworkTableInstance Inst;
  public NetworkTable Table;
  public NetworkTableEntry HasTarget;
  public NetworkTableEntry TargetOffsetH;
  public NetworkTableEntry TargetOffsetV;
  public NetworkTableEntry TargetArea;
  public NetworkTableEntry TargetSkew;

  public Tracking() {
    Inst = NetworkTableInstance.getDefault();
    Table = Inst.getTable("limelight");
    HasTarget = Table.getEntry("tv");
    TargetOffsetH = Table.getEntry("tx");
    TargetOffsetV = Table.getEntry("ty");
    TargetArea = Table.getEntry("ta");
    TargetSkew = Table.getEntry("ts");
  }

  @Override
  public void periodic() {
    
  }
}
