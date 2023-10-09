// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Swerve drive joystick command
 * Based on
 * https://www.chiefdelphi.com/uploads/default/original/3X/8/c/8c0451987d09519712780ce18ce6755c21a0acc0.pdf
 * and
 * https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf
 */
public class DefaultDriveCommand extends CommandBase {
  private Supplier<Double> m_yAxisDrive;
  private Supplier<Double> m_xAxisDrive;
  private Supplier<Double> m_rotationAxis;

  /**
   * Creates new drive command based on joystick inputs
  
   * @param xAxisDrive Right joystick axis
   * @param yAxisDrive Foward joystick axis
   * @param rotationAxis Rotation joystick axis
   */
  public DefaultDriveCommand(Supplier<Double> xAxisDrive, Supplier<Double> yAxisDrive,
      Supplier<Double> rotationAxis) {
    m_yAxisDrive = yAxisDrive;
    m_xAxisDrive = xAxisDrive;
    m_rotationAxis = rotationAxis;
    addRequirements(DriveSubsystem.get());
  }

  /**
   * The main body of a command. Called repeatedly while the command is scheduled.
   * Takes joystick inputs, calculatees the wheel angles and speeds, moves the
   * robots with those values, and logs the calculated numbers
   */
  @Override
  public void execute() {
    //Apply deadband to joystick values
    double fwdSpeed = MathUtil.applyDeadband(m_yAxisDrive.get(), ControllerConstants.kDeadzone);
    double strSpeed = MathUtil.applyDeadband(m_xAxisDrive.get(), ControllerConstants.kDeadzone);
    double rotSpeed = MathUtil.applyDeadband(m_rotationAxis.get(), ControllerConstants.kDeadzone);

    //Get field centric speeds per wheel
    //TODO - for robot centric use "0" instead of "DriveSubsystem.get().getHeading()"
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      fwdSpeed, strSpeed, rotSpeed, Rotation2d.fromDegrees(DriveSubsystem.get().getHeading()));

    // Now use this in our kinematics
    SwerveModuleState[] moduleStates = DriveConstants.kKinematics.toSwerveModuleStates(speeds);

    DriveSubsystem.get().setSwerveStates(moduleStates);
  }
}
