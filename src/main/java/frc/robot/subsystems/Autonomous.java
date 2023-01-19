// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Autonomous extends SubsystemBase {
  public Tracking Limelight;
  private SwerveDrive SwerveDrive;
  private SwerveDriveKinematicsConstraint SwerveDriveMaxSpeed;
  private Constraints PIDConstraints;
  private TrajectoryConfig TrajectoryConfig;
  private Trajectory Trajectory;
  private Path TrajectoryPath;
  private String TrajectoryJSON;
  private SwerveControllerCommand SwerveCommand;
  private Subsystem[] RequiredSubsystems;

  public Autonomous() {
    Limelight = new Tracking();
    Trajectory = new Trajectory();
    TrajectoryJSON = "paths/YourPath.wpilib.json";

    // Create a voltage constraint to ensure we don't accelerate too fast
    SwerveDriveMaxSpeed = new SwerveDriveKinematicsConstraint(SwerveDrive.Kinematics, 1);

    PIDConstraints = new Constraints(1, .25);

    // Create config for trajectory
    TrajectoryConfig = new TrajectoryConfig(1, .25).setKinematics(SwerveDrive.Kinematics).addConstraint(SwerveDriveMaxSpeed);

    try {
      TrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(TrajectoryJSON);
      Trajectory = TrajectoryUtil.fromPathweaverJson(TrajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + TrajectoryJSON, ex.getStackTrace());
    }
  }

  public void runAutonomous() {
    //SwerveCommand = new SwerveControllerCommand(Trajectory, SwerveDrive::getPose, SwerveDrive.Kinematics, new PIDController(1, 0, 0), new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, PIDConstraints), desiredRotation, SwerveDrive::getModuleStates, RequiredSubsystems);
    SwerveDrive.setSwerveOutputs();
  }
}
