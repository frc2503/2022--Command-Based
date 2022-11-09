// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.I2C.Port;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANSparkMax.ControlType;

import java.util.concurrent.TimeUnit;

import javax.lang.model.util.ElementScanner6;
import javax.swing.GroupLayout.Group;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.ejml.equation.Variable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.SwerveDrive;

public class Robot extends TimedRobot {
  private Joystick LeftStick;
  private Joystick RightStick;
  private Double RightStickX;
  private Double RightStickY;
  private Double RightStickTwist;
  public SwerveDrive SwerveDrive;

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    LeftStick = new Joystick(1);
    RightStick = new Joystick(2);
    SwerveDrive = new SwerveDrive();
    SwerveDrive.initMotorControllers(1, 5, 2, 6, 3, 7, 4, 8);
    SwerveDrive.setPID(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
  }

  @Override
  public void teleopPeriodic() {
    // Deadzones
    if (Math.abs(RightStick.getX()) < 0.1) {
      RightStickX = 0.0;
    }
    if (Math.abs(RightStick.getY()) < 0.1) {
      RightStickY = 0.0;
    }
    if (Math.abs(RightStick.getRawAxis(3)) < 0.15) {
      RightStickTwist = 0.0;
    }

    SwerveDrive.swerveDrive(RightStickX, (RightStickY * -1), RightStickTwist, (1 - ((RightStick.getZ() + 1) / 2)), (1 - ((LeftStick.getZ() + 1) / 2)));
  }

  //Autonomous right away
  @Override
  public void autonomousInit(){
  }

  //Autonomous repeat
  @Override
  public void autonomousPeriodic(){ 
  }
}
