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

// This class defines objects and variables for each wheel module
public class Wheel extends SubsystemBase {
  // Define objects and variables
  public CANSparkMax Drive;
  public RelativeEncoder DriveEncoder;
  public SparkMaxPIDController DrivePIDController;
  public CANSparkMax Steer;
  public RelativeEncoder SteerEncoder;
  public SparkMaxPIDController SteerPIDController;
  public Translation2d Location;
  public double DistToPos;
  public double DistSpdMod;
  public double PrevRampedWheelSpd;
  public double RampedWheelSpd;

  // Class constructor, initializes all variables for the created object
  // Input the location of each module relative to the center of the robot
  public Wheel(double ModuleLocationX, double ModuleLocationY) {
    Location = new Translation2d(ModuleLocationX, ModuleLocationY);
    DistToPos = 0.0;
    DistSpdMod = 0.0;
    PrevRampedWheelSpd = 0.0;
    RampedWheelSpd = 0.0;
  }

  // Define what the objects "SteerEncoder" and "SteerPIDController" refer to, and initialize them
  public void initEncodersAndPIDControllers() {
    // Define what encoder the objects "SteerEncoder" and "DriveEncoder" refer to
    DriveEncoder = Drive.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    SteerEncoder = Steer.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 28);
    //SteerEncoder = Steer.getAnalog(SparkMaxAnalogSensor.AnalogMode.kAbsolute);

    // Zero relative encoders, just in case
    DriveEncoder.setPosition(0);
    SteerEncoder.setPosition(0);

    // Purposefully set high to make PID controllers more accurate. If changed, the EncoderPosMod must be changed too. 
    //SteerEncoder.setPositionConversionFactor(50);
    
    // Define what PID controllers the objects "SteerPIDController" and "DrivePIDController" refer to
    DrivePIDController = Drive.getPIDController();
    SteerPIDController = Steer.getPIDController();


    // Set max and min values to be sent to the motors by the PID controllers. Likely shouldn't be changed.
    DrivePIDController.setOutputRange(-1, 1);
    SteerPIDController.setOutputRange(-1, 1);
  }

  // Set the P, I, and D values for the "SteerPIDController" object
  public void setPIDValues(Double P, Double I, Double D) {
    SteerPIDController.setP(P);
    SteerPIDController.setI(I);
    SteerPIDController.setD(D);
  }
}
