// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;


/**
 * Used to rotate the robot
 */
public class AutoTurnCommand extends CommandBase {

  private final double m_targetAngle;    
  private final double m_turnSpeed;    
  private final double m_angleThreshold;  




  /**
   * Creates a new {@code AutoTurnCommand} with the given params.
   * @param targetAngle the angle to turn to (0 is away from you)
   */
  public AutoTurnCommand(double targetAngle, double turnSpeed, double angleThreshold) {
		m_targetAngle = targetAngle;
    m_turnSpeed = turnSpeed;
    m_angleThreshold = angleThreshold;

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
    double speed = m_turnSpeed;
    //Turn the other way if it has passed the goal
    if(DriveSubsystem.get().getHeading() > m_targetAngle){
      speed = -speed;
    }

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      0, 0, speed, Rotation2d.fromDegrees(DriveSubsystem.get().getHeading()));
    // Now use this in our kinematics
    SwerveModuleState[] moduleStates = DriveConstants.kKinematics.toSwerveModuleStates(speeds);

    DriveSubsystem.get().setSwerveStates(moduleStates);
  }


  /**
   * Checks if the turn command should be completed
   * @return {@code true} if the robot has reached its goal;
   *   {@code false} otherwise.
   */
	@Override
	public boolean isFinished() {
    return Math.abs(DriveSubsystem.get().getHeading() - m_targetAngle) <= m_angleThreshold;
  }
}

