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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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
  public SwerveModuleState ModuleState;
  public double DiffToAng;
  public double AngSpdMod;
  public double PrevRampedWheelSpd;
  public double RampedWheelSpd;

  // Class constructor, initializes all variables for the created object
  // Input the location of each module relative to the center of the robot
  public Wheel(double ModuleLocationX, double ModuleLocationY) {
    Location = new Translation2d(ModuleLocationX, ModuleLocationY);
    DiffToAng = 0.0;
    AngSpdMod = 0.0;
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

  // Do swerve drive math that each wheel has to call, pass in EncoderPosMod and DriveRampValue variables
  public void swerveDriveMath(double EncoderPosMod, double DriveRampValue) {
    // Optimize rotation positions, so the wheels don't turn 180 degrees rather than just spinning the drive motor backwards
    ModuleState = SwerveModuleState.optimize(ModuleState, new Rotation2d((SteerEncoder.getPosition() / EncoderPosMod)));

    // Find the differance between the desired wheel angle and the current wheel angle
    DiffToAng = ((Math.abs((SteerEncoder.getPosition() / EncoderPosMod) - ((ModuleState.angle.getDegrees() / 360.0)))));
    
    // Math to make the modifier 1 when the current wheel angle is the same as the desired wheel angle, and 0 at the furthest point away.
    // Original value is multiplied by 4 because, due to angle optimization, the max value the DistToPos variable should be able to reach is .25
    DiffToAng = (1 - (4 * DiffToAng));
    
    // Make absolutely sure the DistToPos variable is greater than or equal to 0, jsut in case the code does something dumb
    if (DiffToAng < 0) {
      DiffToAng = 0;
    }

    // Use the DistToPos variable to create a DistSpdMod variable
    // Used to ramp speed up to the desired speed exponentially as the wheel gets closer to the desired angle
    AngSpdMod = Math.pow(DiffToAng, 5);
    
    // Set the PrevRampedWheelSpd variable to the speed the motors were set to the last time the code was run
    PrevRampedWheelSpd = RampedWheelSpd;
    
    // Set the RampedWheelSpd variable to the desired wheel speed
    // This is done so the speed is still set even if the following if statements return false
    RampedWheelSpd = ((ModuleState.speedMetersPerSecond / 2) * AngSpdMod);
  
    // Determine if the difference in current speed and desired speed is greater than the maximum desired difference (the DriveRampValue variable)
    // If the difference is greater than the maximum desired difference, then find out if the change is greater than or less than zero
    // If the change is greater than zero, then add the maximum desired difference to the previous speed and set that to the new desired speed
    // If the change is less than zero, then subtract the maximum desired difference from the previous speed and set that to the new desired speed
    if (Math.abs(RampedWheelSpd - PrevRampedWheelSpd) > DriveRampValue) {
      if ((RampedWheelSpd - PrevRampedWheelSpd) > 0) {
        RampedWheelSpd = (PrevRampedWheelSpd + DriveRampValue);
      }
      if ((RampedWheelSpd - PrevRampedWheelSpd) < 0) {
        RampedWheelSpd = (PrevRampedWheelSpd - DriveRampValue);
      }
    }
  }
}
