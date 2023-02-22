// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.CounterWeightSubsystem;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.WeightConstants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class CounterWeightCommand extends CommandBase {
  private final CounterWeightSubsystem m_counterWeightSubsystem;
  private CANSparkMax m_counterWeightMotor = new CANSparkMax(WeightConstants.kCANID, MotorType.kBrushless);
  private Supplier<Double> m_xAxisDrive;
  private AHRS navx;
  private final double Kp = 0.03;

  public CounterWeightCommand(CounterWeightSubsystem counterWeightSubsystem, Supplier<Double> xAxisDrive) {
    m_counterWeightSubsystem = counterWeightSubsystem;
    m_xAxisDrive = xAxisDrive;
    addRequirements(m_counterWeightSubsystem);
  }

  @Override
  public void initialize() {
    navx = new AHRS(SPI.Port.kMXP);
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

    double pitch = navx.getPitch();
    double target = 0; // TODO reset pitch
    double error = pitch - target;


    //TODO check this bc it's definitely wrong ESPECIALLY the speed!!!!!!!!!!!!!!!!!!!!!!! :c
    // dean sucks
    if (error > 2) {
      m_counterWeightMotor.set(speed);
    } else if (error < -2) {
      m_counterWeightMotor.set(-speed);
    } else {
      m_counterWeightMotor.set(0);
    }

    // Move the robot
    m_counterWeightSubsystem.setDriveMotors(speed);
  }
}