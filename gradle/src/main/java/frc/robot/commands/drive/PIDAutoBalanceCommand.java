// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BalanceConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;


/**
 * Used to automatically balance the robot on the charge station
 */
public class PIDAutoBalanceCommand extends CommandBase {
  // PID contoller which will drive the bot
  private final PIDController m_controller = new PIDController(BalanceConstants.kP, BalanceConstants.kI, BalanceConstants.kD);


  /**
   * Creates a new {@code PIDAutoBalanceCommand}.
   */
  public PIDAutoBalanceCommand() {
		addRequirements(DriveSubsystem.get());
	}

  /**
   * Called when this command is first scheduled
   */
  @Override
  public void initialize() {
      // Tell the PID controller that it should try to reach 0 degrees (level)
      m_controller.setSetpoint(0);
      // Sets the tolerance
      m_controller.setTolerance(BalanceConstants.kTolerance);
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute() {
    // Calculate the speed the robot should drive
    double speed = m_controller.calculate(DriveSubsystem.get().getPitch());

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      speed, 0, 0.0, Rotation2d.fromDegrees(DriveSubsystem.get().getHeading()));
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
    return m_controller.atSetpoint();
  }

  // Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
    //Stop driving
		DriveSubsystem.get().setDriveMotors(0, 0, 0, 0);
	}
}

