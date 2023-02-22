// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.CounterWeightSubsystem;

public class CounterWeightCommand extends CommandBase {
  private final CounterWeightSubsystem m_counterWeightSubsystem;
  private Supplier<Double> m_xAxisDrive;

  public CounterWeightCommand(CounterWeightSubsystem counterWeightSubsystem, Supplier<Double> xAxisDrive) {
    m_counterWeightSubsystem = counterWeightSubsystem;
    m_xAxisDrive = xAxisDrive;
    addRequirements(m_counterWeightSubsystem);
  }

  @Override
  public void initialize() {
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
    double speed = MathUtil.applyDeadband(m_xAxisDrive.get(), ControllerConstants.kDeadzone);
    // double strSpeed = MathUtil.applyDeadband(m_xAxisDrive.get(), ControllerConstants.kDeadzone);

    // Move the robot
    m_counterWeightSubsystem.setDriveMotors(speed);
  }
}