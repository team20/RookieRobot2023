// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

/**
 * Used the set the arm to a specific position
 *  using a PID control loop. Intended for auto
 */
public class PIDArmCommand extends CommandBase {
    private PIDController m_PIDController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, ArmConstants.kUpdatePeriod);


  public PIDArmCommand(Supplier<Double> targetAngle) {
    m_PIDController.setSetpoint(targetAngle.get());
    addRequirements(ArmSubsystem.get());
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
    double speed = m_PIDController.calculate(ArmSubsystem.get().m_armMotorEncoder.getPosition());
    // Limit the speed of the arm retraction
    // Positive speed is retraction
    if(speed > 0){
      speed *= 0.20;
    }else{
      speed *= 0.65;
    }
    ArmSubsystem.get().setMotorSpeed(speed);
  }

  /**
   * Checks if the arm command should be completed
   * @return {@code true} if the arm has reached its goal;
   *   {@code false} otherwise.
   */
	@Override
	public boolean isFinished() {
    return m_PIDController.atSetpoint();
  }
}