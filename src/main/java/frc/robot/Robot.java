// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.cameraserver.CameraServer;

import frc.robot.subsystems.SwerveDrive;

public class Robot extends TimedRobot {
  // Define objects and variables
  private Joystick LeftStick;
  private Joystick RightStick;
  private Double RightStickX;
  private Double RightStickY;
  private Double RightStickTwist;
  public SwerveDrive SwerveDrive;

  @Override
  public void robotInit() {
    // Start getting video from Limelight
    CameraServer.startAutomaticCapture();
    
    // Assign joysticks to the "LeftStick" and "RightStick" objects
    LeftStick = new Joystick(1);
    RightStick = new Joystick(2);

    // Create an object for the SwerveDrive class
    SwerveDrive = new SwerveDrive();

    // Call SwerveDrive methods, their descriptions are in the SwerveDrive.java file
    SwerveDrive.initMotorControllers(1, 5, 2, 6, 3, 7, 4, 8);
    SwerveDrive.setPID(1.0, 0.0, 0.0);
  }

  @Override
  public void teleopPeriodic() {
    // Assign stick inputs to variables, to prevent discrepancies
    RightStickX = RightStick.getX();
    RightStickY = RightStick.getY();
    RightStickTwist = RightStick.getRawAxis(3);

    // Create deadzones on the joysticks, to prevent stick drift
    if (Math.abs(RightStickX) < 0.1) {
      RightStickX = 0.0;
    }
    if (Math.abs(RightStickY) < 0.1) {
      RightStickY = 0.0;
    }
    if (Math.abs(RightStickTwist) < 0.2) {
      RightStickTwist = 0.0;
    }

    // Call swerveDrive() method, to do all the math and outputs for swerve drive
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
