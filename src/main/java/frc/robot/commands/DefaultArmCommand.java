// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.ArmSubsystem;

public class DefaultArmCommand extends CommandBase {
  private final ArmSubsystem m_armSubsystem;
  private Supplier<Double> m_yAxisDrive;
//   private Supplier<Double> m_xAxisDrive;
//   private Supplier<Double> m_rotationAxis;

  public DefaultArmCommand(ArmSubsystem armSubsystem, Supplier<Double> yAxisDrive
  //, Supplier<Double> rotationAxis
  ) {
    m_armSubsystem = armSubsystem;
    m_yAxisDrive = yAxisDrive;
    // m_xAxisDrive = xAxisDrive;
    // m_rotationAxis = rotationAxis;
    addRequirements(m_armSubsystem);
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
    double speed = MathUtil.applyDeadband(m_yAxisDrive.get(), ControllerConstants.kDeadzone);
    m_armSubsystem.setMotorSpeed(speed);
  }
}