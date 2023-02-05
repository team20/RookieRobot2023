// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Swerve drive joystick command
 * Based on
 * https://www.chiefdelphi.com/uploads/default/original/3X/8/c/8c0451987d09519712780ce18ce6755c21a0acc0.pdf
 * and
 * https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf
 */
public class DefaultDriveCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private Supplier<Double> m_yAxisDrive;
  private Supplier<Double> m_xAxisDrive;
  private Supplier<Double> m_rotationAxis;
  private double m_trackWidth;
  private double m_wheelBase;

  public DefaultDriveCommand(DriveSubsystem driveSubsystem, Supplier<Double> xAxisDrive, Supplier<Double> yAxisDrive,
      Supplier<Double> rotationAxis) {
    m_driveSubsystem = driveSubsystem;
    m_yAxisDrive = yAxisDrive;
    m_xAxisDrive = xAxisDrive;
    m_rotationAxis = rotationAxis;
    addRequirements(m_driveSubsystem);
  }

  @Override
  public void initialize() {
    m_trackWidth = DriveConstants.kTrackWidth;
    m_wheelBase = DriveConstants.kWheelBase;
  }

  /**
   * The main body of a command. Called repeatedly while the command is scheduled.
   * Takes joystick inputs, calculatees the wheel angles and speeds, moves the
   * robots with those values, and logs the calculated numbers
   */
  @Override
  public void execute() {
    // Get the foward, strafe, and rotation speed, using a deadband on the joystick
    // input so slight movements don't move the robot
    double fwdSpeed = MathUtil.applyDeadband(m_yAxisDrive.get(), ControllerConstants.kDeadzone);
    double strSpeed = MathUtil.applyDeadband(m_xAxisDrive.get(), ControllerConstants.kDeadzone);
    double rotSpeed = MathUtil.applyDeadband(m_rotationAxis.get(), ControllerConstants.kDeadzone);
<<<<<<< HEAD

    double leftStickMagnitude = Math.sqrt( fwdSpeed * fwdSpeed + strSpeed * strSpeed);
    double leftStickAngle = Math.atan2(fwdSpeed, strSpeed);
    /* 
    // Random intermediate math
=======
    
    
    // Orig Random intermediate math
>>>>>>> 37727e952d035694146cf8f70f6e60a4ab044625
    double a = strSpeed - rotSpeed * (m_wheelBase / 2);
    double b = strSpeed + rotSpeed * (m_wheelBase / 2);
    double c = fwdSpeed - rotSpeed * (m_trackWidth / 2);
    double d = fwdSpeed + rotSpeed * (m_trackWidth / 2);
<<<<<<< HEAD
    */

    float Deg2Rad = 0.0174532924F;
    // The y component of the FL and FR wheels
    double b = (Math.abs(leftStickMagnitude) * Math.sin(leftStickAngle - m_driveSubsystem.getHeading()) - rotSpeed * Math.sin(Deg2Rad * -135));
    // The y component of the BL and BR wheels
    double a = (Math.abs(leftStickMagnitude) * Math.sin(leftStickAngle - m_driveSubsystem.getHeading()) - rotSpeed * Math.sin(Deg2Rad * -225));
    //The x component of the FL and BL
    double d = (leftStickMagnitude * Math.cos(leftStickAngle - m_driveSubsystem.getHeading()) - rotSpeed * Math.cos(Deg2Rad * -135));
    //The x component of the FR and BR
    double c = (leftStickMagnitude * Math.cos(leftStickAngle - m_driveSubsystem.getHeading()) - rotSpeed * Math.cos(Deg2Rad * -45));
=======

>>>>>>> 37727e952d035694146cf8f70f6e60a4ab044625

    // Calculate the wheel speeds
    double frontRightSpeed = Math.sqrt(b * b + c * c);
    double frontLeftSpeed = Math.sqrt(b * b + d * d);
    double backRightSpeed = Math.sqrt(a * a + c * c);
    double backLeftSpeed = Math.sqrt(a * a + d * d);
    // Initalizing the highest speed, saves one if statement
    double highestSpeed = frontRightSpeed;
    // Normalize wheel speeds if they are calculated to be over 1
    if (frontRightSpeed > 1 || frontLeftSpeed > 1 || backRightSpeed > 1 || backLeftSpeed > 1) {
      // Get the highest speed
      if (highestSpeed < frontLeftSpeed) {
        highestSpeed = frontLeftSpeed;
      }
      if (highestSpeed < backRightSpeed) {
        highestSpeed = backRightSpeed;
      }
      if (highestSpeed < backLeftSpeed) {
        highestSpeed = backLeftSpeed;
      }
      // Normalize all speeds so the fastest one is set to 1
      frontRightSpeed /= highestSpeed;
      frontLeftSpeed /= highestSpeed;
      backRightSpeed /= highestSpeed;
      backLeftSpeed /= highestSpeed;
    }
    // Calculate the wheel angles in degrees
    double flOffset = (132.92 * ((Math.abs(rotSpeed) > 0.05) ? 1 : 0));
    double frOffset = (127.31 * ((Math.abs(rotSpeed) > 0.05) ? 1 : 0));
    double blOffset = (147.31 * ((Math.abs(rotSpeed) > 0.05) ? 1 : 0));
    double brOffset = (91.14 * ((Math.abs(rotSpeed) > 0.05) ? 1 : 0));

    double frontLeftAngle = Math.toDegrees(Math.atan2(b, d) + flOffset);
    double frontRightAngle = Math.toDegrees(Math.atan2(b, c) + frOffset);
    double backRightAngle = Math.toDegrees(Math.atan2(a, c)) + brOffset;
    double backLeftAngle = Math.toDegrees(Math.atan2(a, d) + blOffset);

    
    // SmartDashboard logging
    {
      SmartDashboard.putNumber("Foward Speed", fwdSpeed);
      SmartDashboard.putNumber("Strafe Speed", strSpeed);
      SmartDashboard.putNumber("Rotation Speed", rotSpeed);
      SmartDashboard.putNumber("a", a);
      SmartDashboard.putNumber("b", b);
      SmartDashboard.putNumber("c", c);
      SmartDashboard.putNumber("d", d);
      SmartDashboard.putNumber("Front Right Wheel Speed", frontRightSpeed);
      SmartDashboard.putNumber("Front Left Wheel Speed", frontLeftSpeed);
      SmartDashboard.putNumber("Back Right Wheel Speed", backRightSpeed);
      SmartDashboard.putNumber("Back Left Wheel Speed", backLeftSpeed);
      SmartDashboard.putNumber("Front Right Wheel Angle", frontRightAngle);
      SmartDashboard.putNumber("Front Left Wheel Angle", frontLeftAngle);
      SmartDashboard.putNumber("Back Right Wheel Angle", backRightAngle);
      SmartDashboard.putNumber("Back Left Wheel Angle", backLeftAngle);
    }

<<<<<<< HEAD
    if(fwdSpeed > 0.1 || fwdSpeed < -0.1 || strSpeed > 0.1 || strSpeed < -0.1){
=======
    if(Math.abs(fwdSpeed) + Math.abs(strSpeed) + Math.abs(rotSpeed) > 0.1){
>>>>>>> 37727e952d035694146cf8f70f6e60a4ab044625
      // Move the robot
      m_driveSubsystem.setSteerMotors(frontLeftAngle, frontRightAngle, backLeftAngle, backRightAngle);
    }
    m_driveSubsystem.setDriveMotors(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
  }
}
