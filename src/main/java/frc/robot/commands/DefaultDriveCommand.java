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

    double leftStickMagnitude = Math.sqrt( fwdSpeed * fwdSpeed + strSpeed * strSpeed);
    double leftStickAngle = Math.atan2(strSpeed, fwdSpeed);
    
    // NavX returns gyro andle in degrees
    // Calculations below need radians
    double gyroAngleRad = 0;     // Math.toRadians(m_driveSubsystem.getNavx().getYaw());

    
    // BL & BR x component
    double A = strSpeed - 0.5*leftStickAngle*m_wheelBase;
    // FL & FR x component
    double B = strSpeed + 0.5*leftStickAngle*m_wheelBase;
    // BR & FR y component
    double C = fwdSpeed - 0.5*leftStickAngle*m_wheelBase;
    // FL & FR y component
    double D = fwdSpeed + 0.5*leftStickAngle*m_wheelBase;



    // Calculate the wheel speeds
    double frontRightSpeed = Math.sqrt(B * B + C * C);
    double frontLeftSpeed = Math.sqrt(B * B + D * D);
    double backLeftSpeed = Math.sqrt(A * A + D * D);
    double backRightSpeed = Math.sqrt(A * A + C * C);

    // Initalizing the highest speed, saves one if statement
    double highestSpeed = frontLeftSpeed;
    highestSpeed = Math.max(highestSpeed, Math.abs(frontRightSpeed));
    highestSpeed = Math.max(highestSpeed, Math.abs(backLeftSpeed));
    highestSpeed = Math.max(highestSpeed, Math.abs(backRightSpeed));

      // Normalize all speeds to the max speed so nothing is > 1
    if (highestSpeed > 0) {
      frontRightSpeed /= highestSpeed;
      frontLeftSpeed /= highestSpeed;
      backRightSpeed /= highestSpeed;
      backLeftSpeed /= highestSpeed;
    }
    
    double frontRightAngle = Math.toDegrees(Math.atan2(B, C));
    double frontLeftAngle = Math.toDegrees(Math.atan2(B, D));
    double backLeftAngle = Math.toDegrees(Math.atan2(A, D));
    double backRightAngle = Math.toDegrees(Math.atan2(A, C));

    
    // SmartDashboard logging
    {
      SmartDashboard.putNumber("Foward Speed", fwdSpeed);
      SmartDashboard.putNumber("Strafe Speed", strSpeed);
      SmartDashboard.putNumber("Rotation Speed", rotSpeed);
      SmartDashboard.putNumber("a", A);
      SmartDashboard.putNumber("b", B);
      SmartDashboard.putNumber("c", C);
      SmartDashboard.putNumber("d", D);
      SmartDashboard.putNumber("Front Right Wheel Speed", frontRightSpeed);
      SmartDashboard.putNumber("Front Left Wheel Speed", frontLeftSpeed);
      SmartDashboard.putNumber("Back Right Wheel Speed", backRightSpeed);
      SmartDashboard.putNumber("Back Left Wheel Speed", backLeftSpeed);
      SmartDashboard.putNumber("Front Right Wheel Angle", frontRightAngle);
      SmartDashboard.putNumber("Front Left Wheel Angle", frontLeftAngle);
      SmartDashboard.putNumber("Back Right Wheel Angle", backRightAngle);
      SmartDashboard.putNumber("Back Left Wheel Angle", backLeftAngle);
      SmartDashboard.putNumber("NavX yaw angle", Math.toDegrees(gyroAngleRad));

    }

    // Only set sterring if the wheels are moving  
    if(leftStickMagnitude + Math.abs(rotSpeed) > 0.1)
      m_driveSubsystem.setSteerMotors(frontLeftAngle, frontRightAngle, backLeftAngle, backRightAngle);
    m_driveSubsystem.setDriveMotors(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
  }
}
