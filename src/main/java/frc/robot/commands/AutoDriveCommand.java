// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Swerve drive joystick command
 * Based on
 * https://www.chiefdelphi.com/uploads/default/original/3X/8/c/8c0451987d09519712780ce18ce6755c21a0acc0.pdf
 * and
 * https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf
 */


/**
 * Used to drive the robot a certain distance
 * in a certain directions
 */
public class AutoDriveCommand extends CommandBase {

  /**
   * The distance the robot should drive
   */
  double m_distance;  
  /**
   * Strafe velocity
   */
  double m_speedX;
  /**
   * Foward velocity
   */
  double m_speedY;
  /**
   * Threshold
   */
	double m_distanceThreshold;


  /**
   * Creates a new {@code AutoDriveCommand} with the given params.
   * @param xSpeed The horizontal velocity (left & right relative to the field) 
   * @param ySpeed Foward or backwards velocity
   * @param distance The distance to travel in meters
   * @param distanceThreshold The threshold to exit
   */
  public AutoDriveCommand(double xSpeed, double ySpeed, double distance, double distanceThreshold) {
		m_distance = distance;
    m_speedX = xSpeed;
    m_speedY = ySpeed;
		m_distanceThreshold = distanceThreshold;


		addRequirements(DriveSubsystem.get());
	}

  /**
   * Called when this command is first scheduled
   */
  @Override
  public void initialize() {
      DriveSubsystem.get().getFrontLeftSwerveModule().m_driveEncoder.setPosition(0);
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute() {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      m_speedY, m_speedX, 0.0, Rotation2d.fromDegrees(DriveSubsystem.get().getHeading()));
    // Now use this in our kinematics
    SwerveModuleState[] moduleStates = DriveConstants.kKinematics.toSwerveModuleStates(speeds);

    DriveSubsystem.get().setSwerveStates(moduleStates);
  }


  /**
   * Checks if the drive command should be completed
   * @return {@code true} if the robot has reached its goal;
   *   {@code false} otherwise.
   */
	@Override
	public boolean isFinished() {
    return Math.abs(DriveSubsystem.get().getFrontLeftSwerveModule().m_driveEncoder.getPosition() - m_distance) <= m_distanceThreshold;
  }

  
}

