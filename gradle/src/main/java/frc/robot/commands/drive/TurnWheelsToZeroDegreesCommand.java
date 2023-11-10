// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnWheelsToZeroDegreesCommand extends CommandBase {
  private DriveSubsystem m_driveSubsystem;

  public TurnWheelsToZeroDegreesCommand() {
    m_driveSubsystem = DriveSubsystem.get();
  }

  @Override
  public void initialize() {
    m_driveSubsystem.setWheelRotationToZeroDegrees();
  }
}