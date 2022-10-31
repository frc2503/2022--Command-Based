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
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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

import frc.robot.subsystems.SwerveDrive.Wheel;
import frc.robot.subsystems.SwerveDrive;

public class Robot extends TimedRobot {
  private Joystick LeftStick;
  private Joystick RightStick;
  private boolean HasBeenRun;
  private CANSparkMax ShooterTop;
  private CANSparkMax ShooterBottom;
  private CANSparkMax ArmExtend;
  private Timer timer;
  private final Compressor Compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid Solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  SwerveDrive SwerveDrive = new SwerveDrive();
  Wheel FrontRight = SwerveDrive.FrontRight;
  Wheel FrontLeft = SwerveDrive.FrontLeft;
  Wheel BackLeft = SwerveDrive.BackLeft;
  Wheel BackRight = SwerveDrive.BackRight;

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    timer = new Timer();
    LeftStick = new Joystick(1);
    RightStick = new Joystick(2);

    ShooterBottom = new CANSparkMax(9, MotorType.kBrushless);
    ShooterTop = new CANSparkMax(10, MotorType.kBrushless);
    ArmExtend = new CANSparkMax(11, MotorType.kBrushed);

    Solenoid.set(Value.kOff);
    Compressor.enableDigital();

    SwerveDrive.initMotorControllers(1, 5, 2, 6, 3, 7, 4, 8);
    SwerveDrive.setPID(1.0, 0.0, 0.0);
  }

  @Override
  public void teleopPeriodic() {
    SwerveDrive.swerveDrive(RightStick.getX(), RightStick.getY(), RightStick.getRawAxis(3), RightStick.getZ(), LeftStick.getZ());
    
    if (LeftStick.getRawButton(1)){
      ShooterTop.set(-.6);
      ShooterBottom.set(-.5);
    }
    else{
      if(LeftStick.getRawButton(4)){
        ShooterTop.set(-.6);
      } 
      else if(LeftStick.getRawButton(6)){
        ShooterTop.set(.6);
      }
      else{
        ShooterTop.set(0);
      }
      if(RightStick.getRawButton(3)){
        ShooterBottom.set(-.5);
      }
      else if(RightStick.getRawButton(5)){
        ShooterBottom.set(.5);
      }
      else{
        ShooterBottom.set(0);
      }
    }

    if(LeftStick.getRawButton(5) && !LeftStick.getRawButton(3)) {
      Solenoid.set(Value.kForward);
    } else if(LeftStick.getRawButton(3) && !LeftStick.getRawButton(5)) {
      Solenoid.set(Value.kReverse);
    } else {
      Solenoid.set(Value.kOff);
    }

    if(LeftStick.getRawButtonPressed(2))
      Compressor.disable();
    if(RightStick.getRawButtonPressed(2))
      Compressor.enableDigital();

    if(RightStick.getRawButton(4)){
      ArmExtend.set(1);
    }
    else if(RightStick.getRawButton(6)){
      ArmExtend.set(-1);
    }
    else{
      ArmExtend.set(0);
    }
  }

  //Autonomous right away
  @Override
  public void autonomousInit(){
    HasBeenRun = false;
  }

  //Autonomous repeat
  @Override
  public void autonomousPeriodic(){ 
    FrontRight.SteerPIDController.setReference(0, ControlType.kPosition);
    FrontLeft.SteerPIDController.setReference(0, ControlType.kPosition);
    BackLeft.SteerPIDController.setReference(0, ControlType.kPosition);
    BackRight.SteerPIDController.setReference(0, ControlType.kPosition);
    
    if (HasBeenRun != true){
      timer.reset();
      timer.start();
      HasBeenRun = true;
    }
    while (timer.get() < .5){
      FrontRight.Drive.set(0);
      FrontLeft.Drive.set(0);
      BackLeft.Drive.set(0);
      BackRight.Drive.set(0);
      ShooterTop.set(-.6);
      ShooterBottom.set(-.5);
    }
    while (.5 < timer.get() & timer.get() < 3){
      FrontRight.Drive.set(.25);
      FrontLeft.Drive.set(.25);
      BackLeft.Drive.set(.25);
      BackRight.Drive.set(.25);
      ShooterTop.set(0);
      ShooterBottom.set(-.5);
      //ShooterBottom.set(0);
    }
    while (3 < timer.get() & timer.get() < 4){
      FrontRight.Drive.set(0);
      FrontLeft.Drive.set(0);
      BackLeft.Drive.set(0);
      BackRight.Drive.set(0);
      ShooterBottom.set(-.5);
      //ShooterBottom.set(0);
    }
    while (4 < timer.get() & timer.get() < 6.5){
      FrontRight.Drive.set(-.25);
      FrontLeft.Drive.set(-.25);
      BackLeft.Drive.set(-.25);
      BackRight.Drive.set(-.25);
      ShooterBottom.set(0);
    }
    while (6.5 < timer.get() & timer.get() < 10){
      FrontRight.Drive.set(0);
      FrontLeft.Drive.set(0);
      BackLeft.Drive.set(0);
      BackRight.Drive.set(0);
      ShooterTop.set(-.6);
      ShooterBottom.set(-.5);
    }
    while (10 < timer.get() & timer.get() < 11){
      FrontRight.Drive.set(0);
      FrontLeft.Drive.set(0);
      BackLeft.Drive.set(0);
      BackRight.Drive.set(0);
      ShooterTop.set(0);
      ShooterBottom.set(0);
    }
  }
}