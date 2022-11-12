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

  ChassisSpeeds Speeds;

  public Wheel FrontRight;
  public Wheel FrontLeft;
  public Wheel BackLeft;
  public Wheel BackRight;

  // Class constructor, initializes all objects and variables for the created object
  public SwerveDrive() {
    // Create objects for the Wheel class, and define locations of wheel modules compared to robot center, which doesn't really matter unless base is MUCH longer on one side
    FrontRight = new Wheel(0.381, -0.381);
    FrontLeft = new Wheel(0.381, 0.381);
    BackLeft = new Wheel(-0.381, -0.381);
    BackRight = new Wheel(-0.381, 0.381);

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
    Odometry = new SwerveDriveOdometry(Kinematics, Gyro.getRotation2d(), new Pose2d(0, 0, Gyro.getRotation2d()));
    
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

  // Call the setPIDValues() method for each wheel module, pass in the desired P, I, and D values
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
    SwerveModuleState[] ModuleStates = Kinematics.toSwerveModuleStates(Speeds);

    // Front left module state
    SwerveModuleState frontLeft = ModuleStates[0];

    // Front right module state
    SwerveModuleState frontRight = ModuleStates[1];

    // Back left module state
    SwerveModuleState backLeft = ModuleStates[2];

    // Back right module state
    SwerveModuleState backRight = ModuleStates[3];

    // Update Odometry, so the robot knows its position on the field
    // This section currently only exists so we can use odometry, which solves many other issues, however it will likely be useful for autonomous movement.
    Odometry.update(GyroRotation2d.unaryMinus(),
    new SwerveModuleState(FrontLeft.DriveEncoder.getVelocity(), new Rotation2d(FrontLeft.SteerEncoder.getPosition() / EncoderPosMod)),
    new SwerveModuleState(FrontRight.DriveEncoder.getVelocity(), new Rotation2d(FrontRight.SteerEncoder.getPosition() / EncoderPosMod)),
    new SwerveModuleState(BackLeft.DriveEncoder.getVelocity(), new Rotation2d(BackLeft.SteerEncoder.getPosition() / EncoderPosMod)),
    new SwerveModuleState(BackRight.DriveEncoder.getVelocity(), new Rotation2d(BackRight.SteerEncoder.getPosition() / EncoderPosMod)));
    
    // Optimize rotation positions, so the wheels don't turn 180 degrees rather than just spinning the drive motor backwards
    var frontLeftOptimized = SwerveModuleState.optimize(frontLeft, new Rotation2d((FrontLeft.SteerEncoder.getPosition() / EncoderPosMod)));
    var frontRightOptimized = SwerveModuleState.optimize(frontRight, new Rotation2d((FrontRight.SteerEncoder.getPosition() / EncoderPosMod)));
    var backLeftOptimized = SwerveModuleState.optimize(backLeft, new Rotation2d((BackLeft.SteerEncoder.getPosition() / EncoderPosMod)));
    var backRightOptimized = SwerveModuleState.optimize(backRight, new Rotation2d((BackRight.SteerEncoder.getPosition() / EncoderPosMod)));

    // Find the differance between the desired wheel angle and the current wheel angle
    FrontRight.DistToPos = ((Math.abs((FrontRight.SteerEncoder.getPosition() / EncoderPosMod) - ((frontRightOptimized.angle.getDegrees() / 360.0)))));
    FrontLeft.DistToPos = ((Math.abs((FrontLeft.SteerEncoder.getPosition() / EncoderPosMod) - ((frontLeftOptimized.angle.getDegrees() / 360.0)))));
    BackLeft.DistToPos = ((Math.abs((BackLeft.SteerEncoder.getPosition() / EncoderPosMod) - ((backLeftOptimized.angle.getDegrees() / 360.0)))));
    BackRight.DistToPos = ((Math.abs((BackRight.SteerEncoder.getPosition() / EncoderPosMod) - ((backRightOptimized.angle.getDegrees() / 360.0)))));

    // Math to make the modifier 1 when the current wheel angle is the same as the desired wheel angle, and 0 at the furthest point away.
    // Original value is multiplied by 4 because, due to angle optimization, the max value the DistToPos variable should be able to reach is .25
    FrontRight.DistToPos = (1 - (4 * FrontRight.DistToPos));
    FrontLeft.DistToPos = (1 - (4 * FrontLeft.DistToPos));
    BackLeft.DistToPos = (1 - (4 * BackLeft.DistToPos));
    BackRight.DistToPos = (1 - (4 * BackRight.DistToPos));
    
    // Make absolutely sure the DistToPos variable is greater than or equal to 0, jsut in case the code does something dumb
    if (FrontRight.DistToPos < 0) {
      FrontRight.DistToPos = 0;
    }
    if (FrontLeft.DistToPos < 0) {
      FrontLeft.DistToPos = 0;
    }
    if (BackLeft.DistToPos < 0) {
      BackLeft.DistToPos = 0;
    }
    if (BackRight.DistToPos < 0) {
      BackRight.DistToPos = 0;
    }

    // Use the DistToPos variable to create a DistSpdMod variable
    // Used to ramp speed up to the desired speed exponentially as the wheel gets closer to the desired angle
    FrontRight.DistSpdMod = Math.pow(FrontRight.DistToPos, 5);
    FrontLeft.DistSpdMod = Math.pow(FrontLeft.DistToPos, 5);
    BackLeft.DistSpdMod = Math.pow(BackLeft.DistToPos, 5);
    BackRight.DistSpdMod = Math.pow(BackRight.DistToPos, 5);
    
    // Set the PrevRampedWheelSpd variable to the speed the motors were set to the last time the code was run
    FrontRight.PrevRampedWheelSpd = FrontRight.RampedWheelSpd;
    FrontLeft.PrevRampedWheelSpd = FrontLeft.RampedWheelSpd;
    BackLeft.PrevRampedWheelSpd = BackLeft.RampedWheelSpd;
    BackRight.PrevRampedWheelSpd = BackRight.RampedWheelSpd;
    
    // Set the RampedWheelSpd variable to the desired wheel speed
    // This is done so the speed is still set even if the following if statements return false
    FrontRight.RampedWheelSpd = ((frontRightOptimized.speedMetersPerSecond / 2) * FrontRight.DistSpdMod);
    FrontLeft.RampedWheelSpd = ((frontLeftOptimized.speedMetersPerSecond / 2) * FrontLeft.DistSpdMod);
    BackLeft.RampedWheelSpd = ((backLeftOptimized.speedMetersPerSecond / 2) * BackLeft.DistSpdMod);
    BackRight.RampedWheelSpd = ((backRightOptimized.speedMetersPerSecond / 2) * BackRight.DistSpdMod);
    
    // Determine if the difference in current speed and desired speed is greater than the maximum desired difference (the DriveRampValue variable)
    // If the difference is greater than the maximum desired difference, then find out if the change is greater than or less than zero
    // If the change is greater than zero, then add the maximum desired difference to the previous speed and set that to the new desired speed
    // If the change is less than zero, then subtract the maximum desired difference from the previous speed and set that to the new desired speed
    if (Math.abs(FrontRight.RampedWheelSpd - FrontRight.PrevRampedWheelSpd) > DriveRampValue) {
      if ((FrontRight.RampedWheelSpd - FrontRight.PrevRampedWheelSpd) > 0) {
        FrontRight.RampedWheelSpd = (FrontRight.PrevRampedWheelSpd + DriveRampValue);
      }
      if ((FrontRight.RampedWheelSpd - FrontRight.PrevRampedWheelSpd) < 0) {
        FrontRight.RampedWheelSpd = (FrontRight.PrevRampedWheelSpd - DriveRampValue);
      }
    }
    if (Math.abs(FrontLeft.RampedWheelSpd - FrontLeft.PrevRampedWheelSpd) > DriveRampValue) {
      if ((FrontLeft.RampedWheelSpd - FrontLeft.PrevRampedWheelSpd) > 0) {
        FrontLeft.RampedWheelSpd = (FrontLeft.PrevRampedWheelSpd + DriveRampValue);
      }
      if ((FrontLeft.RampedWheelSpd - FrontLeft.PrevRampedWheelSpd) < 0) {
        FrontLeft.RampedWheelSpd = (FrontLeft.PrevRampedWheelSpd - DriveRampValue);
      }
    }
    if (Math.abs(BackLeft.RampedWheelSpd - BackLeft.PrevRampedWheelSpd) > DriveRampValue) {
      if ((BackLeft.RampedWheelSpd - BackLeft.PrevRampedWheelSpd) > 0) {
        BackLeft.RampedWheelSpd = (BackLeft.PrevRampedWheelSpd + DriveRampValue);
      }
      if ((BackLeft.RampedWheelSpd - BackLeft.PrevRampedWheelSpd) < 0) {
        BackLeft.RampedWheelSpd = (BackLeft.PrevRampedWheelSpd - DriveRampValue);
      }
    }
    if (Math.abs(BackRight.RampedWheelSpd - BackRight.PrevRampedWheelSpd) > DriveRampValue) {
      if ((BackRight.RampedWheelSpd - BackRight.PrevRampedWheelSpd) > 0) {
        BackRight.RampedWheelSpd = (BackRight.PrevRampedWheelSpd + DriveRampValue);
      }
      if ((BackRight.RampedWheelSpd - BackRight.PrevRampedWheelSpd) < 0) {
        BackRight.RampedWheelSpd = (BackRight.PrevRampedWheelSpd - DriveRampValue);
      }
    }

    // Tell the steer motors to turn the wheels to the correct position
    // An issue is created by ramping which this if statement solves, I will explain the roots of the problem, and the solution here:
    // If all inputs for robot speeds are 0, the angle for the wheels will default to 0
    // This causes a problem because the drive wheel speed does not instantly go to zero, causing the robot's direction to change
    // This if statement fixes this issue by only changing the angle of the wheels if any of the desired robot speeds are greater than 0
    if ((Math.abs(X) + Math.abs(Y) + Math.abs(Spin)) != 0) {
      FrontRight.SteerPIDController.setReference(((frontRightOptimized.angle.getDegrees() / 360.0) * EncoderPosMod), ControlType.kPosition);
      FrontLeft.SteerPIDController.setReference(((frontLeftOptimized.angle.getDegrees() / 360.0) * EncoderPosMod), ControlType.kPosition);
      BackLeft.SteerPIDController.setReference(((backLeftOptimized.angle.getDegrees() / 360.0) * EncoderPosMod), ControlType.kPosition);
      BackRight.SteerPIDController.setReference(((backRightOptimized.angle.getDegrees() / 360.0) * EncoderPosMod), ControlType.kPosition);
    }
    
    // Tell the drive motors to drive the wheels at the correct speed
    FrontRight.Drive.set(FrontRight.RampedWheelSpd);
    FrontLeft.Drive.set(FrontLeft.RampedWheelSpd);
    BackLeft.Drive.set(BackLeft.RampedWheelSpd);
    BackRight.Drive.set(BackRight.RampedWheelSpd);
  }
}
