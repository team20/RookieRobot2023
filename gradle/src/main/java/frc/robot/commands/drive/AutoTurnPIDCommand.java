// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;


/**
 * Used to rotate the robot
 */
public class AutoTurnPIDCommand extends CommandBase {

  private final double m_targetAngle;    
  private final double m_angleThreshold;  


  private final PIDController m_contoller = new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);


  /**
   * Creates a new {@code AutoTurnPIDCommand} with the given params.
   * @param targetAngle the angle to turn to (0 is away from you)
   * @param angleThreshold the threshold
   */
  public AutoTurnPIDCommand(double targetAngle, double angleThreshold) {
		m_targetAngle = targetAngle;
    m_angleThreshold = angleThreshold;

		addRequirements(DriveSubsystem.get());
	}

  /**
   * Called when this command is first scheduled
   */
  @Override
  public void initialize() {
      m_contoller.setSetpoint(m_targetAngle);
      m_contoller.setTolerance(m_angleThreshold);
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute() {
    double speed = m_contoller.calculate(DriveSubsystem.get().getHeading());

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

  // Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
    //Stop driving
		DriveSubsystem.get().setDriveMotors(0, 0, 0, 0);
	}
}

