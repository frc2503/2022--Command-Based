// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;

import java.lang.reflect.Method;

import javax.swing.plaf.synth.SynthSpinnerUI;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.Wheel;

/**
Class to hold all code for the swerve drive
Mostly exists to prevent people from accidentally messing the code up
*/
public class SwerveDrive extends SubsystemBase {
  // Define all objects and varibles used by this class
  private AHRS Gyro;
  private Rotation2d GyroRotation2d;

  private SwerveDriveKinematics Kinematics;
  private SwerveDriveOdometry Odometry;

  private double EncoderPosMod;
  private double DriveRampValue;

  private ChassisSpeeds Speeds;
  private SwerveModuleState[] ModuleStates;

  public Wheel FrontRight;
  public Wheel FrontLeft;
  public Wheel BackLeft;
  public Wheel BackRight;

  // Class constructor, initializes all objects and variables for the created object
  public SwerveDrive() {
    // Create objects for the Wheel class, and define locations of wheel modules compared to robot center, which doesn't really matter unless base is MUCH longer on one side
    FrontRight = new Wheel(0.355, -0.381);
    FrontLeft = new Wheel(0.355, 0.381);
    BackLeft = new Wheel(-0.355, -0.381);
    BackRight = new Wheel(-0.355, 0.381);

    // Create a ChassisSpeeds object, which we later pass our desired speeds into to get our wheel speeds and angles
    Speeds = new ChassisSpeeds();
    
    // Initialize and zero gyro
    Gyro = new AHRS(I2C.Port.kMXP);
    Gyro.calibrate();
    Gyro.reset();
    
    // Pass in locations of wheels relative to the center of the robot
    // These are later used in the backend, likely to find the angles the wheels need to rotate to when the robot spins 
    Kinematics = new SwerveDriveKinematics(FrontRight.Location, FrontLeft.Location, BackLeft.Location, BackRight.Location);
    
    // Pass in wheel module locations, as well as initial robot angle and position for field oriented drive
    // The robot position is unused for now, but might be utilized in autonomous later
    Odometry = new SwerveDriveOdometry(Kinematics, Gyro.getRotation2d(), new Pose2d(0, 0, new Rotation2d()));
    
    /**
    Number to modify the encoders' output value.
    On relative encoders, it is used to deal with the gear ratio.
    On absoute encoders, it is used to counteract the position conversion factor, which is purposfully set high to make the PID controllers more accurate.
    */
    EncoderPosMod = (59.0 + (1.0/6.0));

    // Amount the drive speed can increas or decrease by, max value of 1, min value of 0
    // Purposefully set very low because of how quickly the code runs
    DriveRampValue = .02;
  }

  // Assign motor controllers their CAN numbers, and call the initEncodersAndPIDControllers() method for each wheel module
  // Pass in the Spark Max CAN bus numbers
  // Call during robotInit().
  public void initMotorControllers(Integer FRD, Integer FRS, Integer FLD, Integer FLS, Integer BLD, Integer BLS, Integer BRD, Integer BRS) {
    FrontRight.Drive = new CANSparkMax(FRD, MotorType.kBrushless);
    FrontRight.Steer = new CANSparkMax(FRS, MotorType.kBrushed);
    FrontLeft.Drive = new CANSparkMax(FLD, MotorType.kBrushless);
    FrontLeft.Steer = new CANSparkMax(FLS, MotorType.kBrushed);
    BackRight.Drive = new CANSparkMax(BRD, MotorType.kBrushless);
    BackRight.Steer = new CANSparkMax(BRS, MotorType.kBrushed);
    BackLeft.Drive = new CANSparkMax(BLD, MotorType.kBrushless);
    BackLeft.Steer = new CANSparkMax(BLS, MotorType.kBrushed);
    
    FrontRight.initEncodersAndPIDControllers();
    FrontLeft.initEncodersAndPIDControllers();
    BackLeft.initEncodersAndPIDControllers();
    BackRight.initEncodersAndPIDControllers();
  }

  // Call the setPIDValues() method for each wheel module
  // Pass in the desired P, I, and D values
  // Call during robotInit(), after initMotorControllers().
  public void setPID(Double P, Double I, Double D) {
    FrontRight.setPIDValues(P, I, D);
    FrontLeft.setPIDValues(P, I, D);
    BackLeft.setPIDValues(P, I, D);
    BackRight.setPIDValues(P, I, D);
  }

  // This method does all of the math for the swerve drive, pass in desired translation speeds, spin speed, and the modifier to put on those speeds
  // Translation and spin speeds on a scale from -1 to 1
  // Modifiers on a scale from 0 to 1
  // Call during teleopPeriodic()
  public void swerveDrive(Double X, Double Y, Double Spin, Double XYMod, Double SpinMod) {
    // Need to set the gyro angle to a variable in order to invert the output
    GyroRotation2d = Gyro.getRotation2d();

    // Set the desired speeds for the robot, we also pass in the gyro angle for field oriented drive
    Speeds = ChassisSpeeds.fromFieldRelativeSpeeds((Y * XYMod), (X * XYMod), (Spin * SpinMod), GyroRotation2d.unaryMinus());

    // Convert overall robot speeds and angle into speeds and angles for each wheel module, referred to as module states
    ModuleStates = Kinematics.toSwerveModuleStates(Speeds);

    // Front left module state
    FrontLeft.ModuleState = ModuleStates[0];

    // Front right module state
    FrontRight.ModuleState = ModuleStates[1];

    // Back left module state
    BackLeft.ModuleState = ModuleStates[2];

    // Back right module state
    BackRight.ModuleState = ModuleStates[3];

    // Update Odometry, so the robot knows its position on the field
    // This section currently only exists so we can use odometry, which solves many other issues, however it will likely be useful for autonomous movement.
    Odometry.update(GyroRotation2d.unaryMinus(),
    new SwerveModuleState(FrontLeft.DriveEncoder.getVelocity(), new Rotation2d(FrontLeft.SteerEncoder.getPosition() / EncoderPosMod)),
    new SwerveModuleState(FrontRight.DriveEncoder.getVelocity(), new Rotation2d(FrontRight.SteerEncoder.getPosition() / EncoderPosMod)),
    new SwerveModuleState(BackLeft.DriveEncoder.getVelocity(), new Rotation2d(BackLeft.SteerEncoder.getPosition() / EncoderPosMod)),
    new SwerveModuleState(BackRight.DriveEncoder.getVelocity(), new Rotation2d(BackRight.SteerEncoder.getPosition() / EncoderPosMod)));
    
    // Do math for swerve drive that is identical between all wheel modules, and then send the angle and speed to the wheels
    // See Wheel.java for full explanations
    FrontRight.swerveDriveSetOutputs(X, Y, Spin, EncoderPosMod, DriveRampValue);
    FrontLeft.swerveDriveSetOutputs(X, Y, Spin, EncoderPosMod, DriveRampValue);
    BackLeft.swerveDriveSetOutputs(X, Y, Spin, EncoderPosMod, DriveRampValue);
    BackRight.swerveDriveSetOutputs(X, Y, Spin, EncoderPosMod, DriveRampValue);
  }
}
