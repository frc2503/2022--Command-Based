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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Translation2d;

public class Wheel extends SubsystemBase {
  public CANSparkMax Drive;
  public RelativeEncoder DriveEncoder;
  public SparkMaxPIDController DrivePIDController;
  public CANSparkMax Steer;
  public RelativeEncoder SteerEncoder;
  public SparkMaxPIDController SteerPIDController;
  public Translation2d Location;
  public double DistToPos;
  public double DistSpdMod;

  public Wheel(double ModuleLocationX, double ModuleLocationY) {
    Location = new Translation2d(ModuleLocationX, ModuleLocationY);
    DistToPos = 0.0;
    DistSpdMod = 0.0;
    RampedWheelSpeed = 0.0;
  }

  public void initEncodersAndPIDControllers() {
    SteerEncoder = Steer.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 28);
    //SteerEncoder = Steer.getAnalog(SparkMaxAnalogSensor.AnalogMode.kAbsolute);
    DriveEncoder = Drive.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    // Zero relative encoders, just in case
    SteerEncoder.setPosition(0);
    DriveEncoder.setPosition(0);

    // Purposefully set high to make PID controllers more accurate. If changed, the EncoderPosMod must be changed too. 
    //SteerEncoder.setPositionConversionFactor(50);
    
    SteerPIDController = Steer.getPIDController();
    DrivePIDController = Drive.getPIDController();

    // Set max and min values to be sent to the motors by the PID controllers. Likely shouldn't be changed.
    SteerPIDController.setOutputRange(-1, 1);
    DrivePIDController.setOutputRange(-1, 1);
  }

  public void setPIDValues(Double PD, Double PS, Double ID, Double IS, Double DD, Double DS) {
    DrivePIDController.setP(PD);
    SteerPIDController.setP(PS);
    DrivePIDController.setP(ID);
    SteerPIDController.setP(IS);
    DrivePIDController.setP(DD);
    SteerPIDController.setP(DS);
  }
}
