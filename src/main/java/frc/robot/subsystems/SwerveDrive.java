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

public class SwerveDrive extends SubsystemBase {
  private AHRS Gyro;
  private Rotation2d GyroRotation2d;

  private SwerveDriveKinematics Kinematics;
  private SwerveDriveOdometry Odometry;

  private double EncoderPosMod;

  ChassisSpeeds Speeds;

  public Wheel FrontRight;
  public Wheel FrontLeft;
  public Wheel BackLeft;
  public Wheel BackRight;

  // Initialize all objects which are unlikely to change
  public SwerveDrive() {
    // Create objects for the Wheel class, and define locations of wheel modules compared to robot center, which doesn't really matter unless base is MUCH longer on one side
    FrontRight = new Wheel(0.381, -0.381);
    FrontLeft = new Wheel(0.381, 0.381);
    BackLeft = new Wheel(-0.381, -0.381);
    BackRight = new Wheel(-0.381, 0.381);

    Speeds = new ChassisSpeeds();
    
    // Define and zero gyro
    Gyro = new AHRS(I2C.Port.kMXP);
    Gyro.calibrate();
    Gyro.reset();
    
    Kinematics = new SwerveDriveKinematics(FrontRight.Location, FrontLeft.Location, BackLeft.Location, BackRight.Location);
    Odometry = new SwerveDriveOdometry(Kinematics, Gyro.getRotation2d(), new Pose2d(5.0, 13.5, new Rotation2d()));
    
    /**
    Number to modify the encoders' output value.
    On relative encoders, it is used to deal with the gear ratio.
    On absoute encoders, it is used to counteract the position conversion factor, which is purposfully set high to make the PID controllers more accurate.
    */
    EncoderPosMod = (59.0 + (1.0/6.0));
  }

  // Assign motor controllers their CAN numbers, and setup all relating things. Call during robotInit().
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

  // Setup PID values. Call during robotInit(), after initMotorControllers().
  public void setPID(Double PD, Double PS, Double ID, Double IS, Double DD, Double DS) {
    FrontRight.setPIDValues(PD, PS, ID, IS, DD, DS);
    FrontLeft.setPIDValues(PD, PS, ID, IS, DD, DS);
    BackLeft.setPIDValues(PD, PS, ID, IS, DD, DS);
    BackRight.setPIDValues(PD, PS, ID, IS, DD, DS);
  }

  // Where the black magic happens. Call during teleopPeriodic()
  public void swerveDrive(Double RSX, Double RSY, Double RST, Double RSZ, Double LSZ) {
    // Need to set the gyro angle to a variable in order to invert the output
    GyroRotation2d = Gyro.getRotation2d();

    // Set ChassisSpeeds for actual movement
    Speeds = ChassisSpeeds.fromFieldRelativeSpeeds((RSY * RSZ), (RSX * RSZ), (RST * LSZ), GyroRotation2d.unaryMinus());

    // Convert to module states
    SwerveModuleState[] ModuleStates = Kinematics.toSwerveModuleStates(Speeds);

    // Front left module state
    SwerveModuleState frontLeft = ModuleStates[0];

    // Front right module state
    SwerveModuleState frontRight = ModuleStates[1];

    // Back left module state
    SwerveModuleState backLeft = ModuleStates[2];

    // Back right module state
    SwerveModuleState backRight = ModuleStates[3];

    // Update Odometry
    Odometry.update(GyroRotation2d.unaryMinus(),
    new SwerveModuleState(FrontLeft.DriveEncoder.getVelocity(), new Rotation2d(FrontLeft.SteerEncoder.getPosition() / EncoderPosMod)),
    new SwerveModuleState(FrontRight.DriveEncoder.getVelocity(), new Rotation2d(FrontRight.SteerEncoder.getPosition() / EncoderPosMod)),
    new SwerveModuleState(BackLeft.DriveEncoder.getVelocity(), new Rotation2d(BackLeft.SteerEncoder.getPosition() / EncoderPosMod)),
    new SwerveModuleState(BackRight.DriveEncoder.getVelocity(), new Rotation2d(BackRight.SteerEncoder.getPosition() / EncoderPosMod)));
    
    // Optimize rotation positions
    var frontLeftOptimized = SwerveModuleState.optimize(frontLeft, new Rotation2d((FrontLeft.SteerEncoder.getPosition() / EncoderPosMod)));
    var frontRightOptimized = SwerveModuleState.optimize(frontRight, new Rotation2d((FrontRight.SteerEncoder.getPosition() / EncoderPosMod)));
    var backLeftOptimized = SwerveModuleState.optimize(backLeft, new Rotation2d((BackLeft.SteerEncoder.getPosition() / EncoderPosMod)));
    var backRightOptimized = SwerveModuleState.optimize(backRight, new Rotation2d((BackRight.SteerEncoder.getPosition() / EncoderPosMod)));

    // Find the distance to the desired wheel angle
    FrontRight.DistToPos = ((Math.abs((FrontRight.SteerEncoder.getPosition() / EncoderPosMod) - ((frontRightOptimized.angle.getDegrees() / 360.0)))));
    FrontLeft.DistToPos = ((Math.abs((FrontLeft.SteerEncoder.getPosition() / EncoderPosMod) - ((frontLeftOptimized.angle.getDegrees() / 360.0)))));
    BackLeft.DistToPos = ((Math.abs((BackLeft.SteerEncoder.getPosition() / EncoderPosMod) - ((backLeftOptimized.angle.getDegrees() / 360.0)))));
    BackRight.DistToPos = ((Math.abs((BackRight.SteerEncoder.getPosition() / EncoderPosMod) - ((backRightOptimized.angle.getDegrees() / 360.0)))));

    // More math to set up for the next calculation
    FrontRight.DistToPos = (1 - (2 * FrontRight.DistToPos));
    FrontLeft.DistToPos = (1 - (2 * FrontLeft.DistToPos));
    BackLeft.DistToPos = (1 - (2 * BackLeft.DistToPos));
    BackRight.DistToPos = (1 - (2 * BackRight.DistToPos));

    // Create modifier to decrease the drive speed of the wheel exponentially the farther it is from the desired angle
    FrontRight.DistSpdMod = Math.pow(FrontRight.DistToPos, 5);
    FrontLeft.DistSpdMod = Math.pow(FrontLeft.DistToPos, 5);
    BackLeft.DistSpdMod = Math.pow(BackLeft.DistToPos, 5);
    BackRight.DistSpdMod = Math.pow(BackRight.DistToPos, 5);

    // Actually tell the wheel to turn to the correct position
    FrontRight.SteerPIDController.setReference(((frontRightOptimized.angle.getDegrees() / 360.0) * EncoderPosMod), ControlType.kPosition);
    FrontLeft.SteerPIDController.setReference(((frontLeftOptimized.angle.getDegrees() / 360.0) * EncoderPosMod), ControlType.kPosition);
    BackLeft.SteerPIDController.setReference(((backLeftOptimized.angle.getDegrees() / 360.0) * EncoderPosMod), ControlType.kPosition);
    BackRight.SteerPIDController.setReference(((backRightOptimized.angle.getDegrees() / 360.0) * EncoderPosMod), ControlType.kPosition);

    // Actually tell the wheel to turn at the correct speed
    FrontRight.Drive.set((frontRightOptimized.speedMetersPerSecond / 2) * FrontRight.DistSpdMod);
    FrontLeft.Drive.set((frontLeftOptimized.speedMetersPerSecond / 2) * FrontLeft.DistSpdMod);
    BackLeft.Drive.set((backLeftOptimized.speedMetersPerSecond / 2) * BackLeft.DistSpdMod);
    BackRight.Drive.set((backRightOptimized.speedMetersPerSecond / 2) * BackRight.DistSpdMod);
    
    //FrontRight.DrivePIDController.setReference((((frontRightOptimized.speedMetersPerSecond * 5000) / 2) * FrontRight.DistSpdMod), ControlType.kVelocity);
    //FrontLeft.DrivePIDController.setReference((((frontLeftOptimized.speedMetersPerSecond * 5000) / 2) * FrontLeft.DistSpdMod), ControlType.kVelocity);
    //BackLeft.DrivePIDController.setReference((((backLeftOptimized.speedMetersPerSecond * 5000) / 2) * BackLeft.DistSpdMod), ControlType.kVelocity);
    //BackRight.DrivePIDController.setReference((((backRightOptimized.speedMetersPerSecond * 5000) / 2) * BackRight.DistSpdMod), ControlType.kVelocity);
    
  }
}
