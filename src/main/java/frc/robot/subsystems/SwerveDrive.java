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

public class SwerveDrive extends SubsystemBase {
  private AHRS ahrs;

  // Locations of wheel modules compared to robot center (doesn't really matter unless base is VERY rectangular)
  private Translation2d FrontRightLocation = new Translation2d(0.381, -0.381);
  private Translation2d FrontLeftLocation = new Translation2d(0.381, 0.381);
  private Translation2d BackLeftLocation = new Translation2d(-0.381, -0.381);
  private Translation2d BackRightLocation = new Translation2d(-0.381, 0.381);

  SwerveDriveKinematics Kinematics = new SwerveDriveKinematics(FrontRightLocation, FrontLeftLocation, BackLeftLocation, BackRightLocation);
  SwerveDriveOdometry Odometry;

  private double EncoderPosMod;

  // Class to give each wheel module its own variables
  public class Wheel {
    public CANSparkMax Drive;
    public RelativeEncoder DriveEncoder;
    public SparkMaxPIDController DrivePIDController;
    public CANSparkMax Steer;
    public RelativeEncoder SteerEncoder;
    public SparkMaxPIDController SteerPIDController;
    public double DistToPos;
    public double DistSpdMod;

    public Wheel(CANSparkMax Drive, RelativeEncoder DriveEncoder, SparkMaxPIDController DrivePIDController, CANSparkMax Steer, RelativeEncoder SteerEncoder, SparkMaxPIDController PIDController, double DistToPos, double DistSpdMod) {
      this.Drive = Drive;
      this.DriveEncoder = DriveEncoder;
      this.DrivePIDController = DrivePIDController;
      this.Steer = Steer;
      this.SteerEncoder = SteerEncoder;
      this.SteerPIDController = SteerPIDController;
      this.DistToPos = DistToPos;
      this.DistSpdMod = DistSpdMod;
    }
  }

  public Wheel FrontRight = new Wheel(null, null, null, null, null, null, 0, 0);
  public Wheel FrontLeft = new Wheel(null, null, null, null, null, null, 0, 0);
  public Wheel BackLeft = new Wheel(null, null, null, null, null, null, 0, 0);
  public Wheel BackRight = new Wheel(null, null, null, null, null, null, 0, 0);

  // Initialize all objects which are unlikely to change
  public SwerveDrive() {
    ahrs = new AHRS(I2C.Port.kMXP);
    Odometry = new SwerveDriveOdometry(Kinematics, ahrs.getRotation2d(), new Pose2d(5.0, 13.5, new Rotation2d()));

    /**
    Number to modify the encoders' output value.
    On relative encoders, it is used to deal with the gear ratio.
    On absoute encoders, it is used to counteract the position conversion factor, which is purposfully set high to make the PID controllers more accurate.
    */
    EncoderPosMod = (59.0 + (1.0/6.0));

    // Zero gyro
    ahrs.calibrate();
    ahrs.reset();
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
    
    FrontRight.SteerEncoder = FrontRight.Steer.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, (28));
    FrontLeft.SteerEncoder = FrontLeft.Steer.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, (28));
    BackLeft.SteerEncoder = BackLeft.Steer.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, (28));
    BackRight.SteerEncoder = BackRight.Steer.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, (28));

    /**
    FrontRight.SteerEncoder = FrontRight.Steer.getAnalog(SparkMaxAnalogSensor.AnalogMode.kAbsolute);
    FrontLeft.SteerEncoder = FrontLeft.Steer.getAnalog(SparkMaxAnalogSensor.AnalogMode.kAbsolute);
    BackLeft.SteerEncoder = BackLeft.Steer.getAnalog(SparkMaxAnalogSensor.AnalogMode.kAbsolute);
    BackRight.SteerEncoder = BackRight.Steer.getAnalog(SparkMaxAnalogSensor.AnalogMode.kAbsolute);
    */

    FrontRight.DriveEncoder = FrontRight.Drive.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, (42));
    FrontLeft.DriveEncoder = FrontLeft.Drive.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, (42));
    BackRight.DriveEncoder = BackRight.Drive.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, (42));
    BackLeft.DriveEncoder = BackLeft.Drive.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, (42));

    // Zero relative encoders, just in case
    FrontRight.SteerEncoder.setPosition(0);
    FrontLeft.SteerEncoder.setPosition(0);
    BackLeft.SteerEncoder.setPosition(0);
    BackRight.SteerEncoder.setPosition(0);
    
    FrontRight.DriveEncoder.setPosition(0);
    FrontLeft.DriveEncoder.setPosition(0);
    BackRight.DriveEncoder.setPosition(0);
    BackLeft.DriveEncoder.setPosition(0);

    /**
    // Purposefully set high to make PID controllers more accurate. If changed, the EncoderPosMod must be changed too. 
    FrontRight.SteerEncoder.setPositionConversionFactor(50);
    FrontLeft.SteerEncoder.setPositionConversionFactor(50);
    BackLeft.SteerEncoder.setPositionConversionFactor(50);
    BackRight.SteerEncoder.setPositionConversionFactor(50);
    */

    FrontRight.SteerPIDController = FrontRight.Steer.getPIDController();
    FrontLeft.SteerPIDController = FrontLeft.Steer.getPIDController();
    BackLeft.SteerPIDController = BackLeft.Steer.getPIDController();
    BackRight.SteerPIDController = BackRight.Steer.getPIDController();

    FrontRight.DrivePIDController = FrontRight.Drive.getPIDController();
    FrontLeft.DrivePIDController = FrontLeft.Drive.getPIDController();
    BackRight.DrivePIDController = BackLeft.Drive.getPIDController();
    BackLeft.DrivePIDController = BackRight.Drive.getPIDController();

    // Set max and min values to be sent to the motors by the PID controllers. Likely shouldn't be changed.
    FrontRight.SteerPIDController.setOutputRange(-1, 1);
    FrontLeft.SteerPIDController.setOutputRange(-1, 1);
    BackLeft.SteerPIDController.setOutputRange(-1, 1);
    BackRight.SteerPIDController.setOutputRange(-1, 1);

    FrontRight.DrivePIDController.setOutputRange(-1, 1);
    FrontLeft.DrivePIDController.setOutputRange(-1, 1);
    BackLeft.DrivePIDController.setOutputRange(-1, 1);
    BackRight.DrivePIDController.setOutputRange(-1, 1);
  }

  // Setup PID values. Call during robotInit(), after initMotorControllers().
  public void setPID(Double P, Double I, Double D) {
    FrontRight.SteerPIDController.setP(P);
    FrontLeft.SteerPIDController.setP(P);
    BackLeft.SteerPIDController.setP(P);
    BackRight.SteerPIDController.setP(P);

    FrontRight.DrivePIDController.setP(P);
    FrontLeft.DrivePIDController.setP(P);
    BackLeft.DrivePIDController.setP(P);
    BackRight.DrivePIDController.setP(P);

    FrontRight.SteerPIDController.setI(I);
    FrontLeft.SteerPIDController.setI(I);
    BackLeft.SteerPIDController.setI(I);
    BackRight.SteerPIDController.setI(I);

    FrontRight.DrivePIDController.setI(I);
    FrontLeft.DrivePIDController.setI(I);
    BackLeft.DrivePIDController.setI(I);
    BackRight.DrivePIDController.setI(I);

    FrontRight.SteerPIDController.setD(D);
    FrontLeft.SteerPIDController.setD(D);
    BackLeft.SteerPIDController.setD(D);
    BackRight.SteerPIDController.setD(D);

    FrontRight.DrivePIDController.setD(D);
    FrontLeft.DrivePIDController.setD(D);
    BackLeft.DrivePIDController.setD(D);
    BackRight.DrivePIDController.setD(D);
  }

  // Where the black magic happens. Call during teleopPeriodic()
  public void swerveDrive(Double RSX, Double RSY, Double RST, Double RSZ, Double LSZ) {
    // Deadzones
    if (Math.abs(RSX) < 0.1) {
      RSX = 0.0;
    }
    if (Math.abs(RSY) < 0.1) {
      RSY = 0.0;
    }
    if (Math.abs(RST) < 0.15) {
      RST = 0.0;
    }
    
    // Set ChassisSpeeds for actual movement
    ChassisSpeeds speeds = new ChassisSpeeds(((RSY * -1) * (1 - ((RSZ + 1) / 2))), (RSX * (1 - ((RSZ + 1) / 2))), (RST * (1 - ((LSZ + 1) / 2))));

    // Convert to module states
    SwerveModuleState[] ModuleStates = Kinematics.toSwerveModuleStates(speeds);

    // Front left module state
    SwerveModuleState frontLeft = ModuleStates[0];

    // Front right module state
    SwerveModuleState frontRight = ModuleStates[1];

    // Back left module state
    SwerveModuleState backLeft = ModuleStates[2];

    // Back right module state
    SwerveModuleState backRight = ModuleStates[3];

    // Update Odometry
    Odometry.update(ahrs.getRotation2d(), new SwerveModuleState(FrontLeft.DriveEncoder.getVelocity(), new Rotation2d(FrontLeft.SteerEncoder.getPosition() / EncoderPosMod)), new SwerveModuleState(FrontRight.DriveEncoder.getVelocity(), new Rotation2d(FrontRight.SteerEncoder.getPosition() / EncoderPosMod)),
    new SwerveModuleState(BackLeft.DriveEncoder.getVelocity(), new Rotation2d(BackLeft.SteerEncoder.getPosition() / EncoderPosMod)), new SwerveModuleState(BackRight.DriveEncoder.getVelocity(), new Rotation2d(BackRight.SteerEncoder.getPosition() / EncoderPosMod)));

    //ModuleStates = Kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(((RightStickY * -1) * RightStickZ), (RightStickX * RightStickZ), (RightStickTwist * LeftStickZ), ahrs.getRotation2d()));
    
    // Optimize rotation positions
    var frontLeftOptimized = SwerveModuleState.optimize(frontLeft,
    new Rotation2d((FrontLeft.SteerEncoder.getPosition() / EncoderPosMod)));
    var frontRightOptimized = SwerveModuleState.optimize(frontRight,
    new Rotation2d((FrontRight.SteerEncoder.getPosition() / EncoderPosMod)));
    var backLeftOptimized = SwerveModuleState.optimize(backLeft,
    new Rotation2d((BackLeft.SteerEncoder.getPosition() / EncoderPosMod)));
    var backRightOptimized = SwerveModuleState.optimize(backRight,
    new Rotation2d((BackRight.SteerEncoder.getPosition() / EncoderPosMod)));

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
  }
}
