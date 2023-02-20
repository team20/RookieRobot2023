// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WeightConstants;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem.BrakePneumatics;
import java.lang.Math;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax m_armMotor = new CANSparkMax(ArmConstants.kCANDICE, MotorType.kBrushless);
  public RelativeEncoder m_armMotorEncoder = m_armMotor.getEncoder();
  private static ArmSubsystem a_subsystem;
  private static PneumaticsSubsystem p_subsystem;
  DigitalInput m_aTopSwitch = new DigitalInput(ArmConstants.kArmTopLimitDIO);
  DigitalInput m_aBotSwitch = new DigitalInput(ArmConstants.kArmBotLimitDIO);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // Singleton
    if (a_subsystem != null) {
      try {
        throw new Exception("Arm subsystem already initalized!");
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
    a_subsystem = this;

     // Initialize motors, PID controllers
    {
      configMotorController(m_armMotor);
    }
  }

  public static ArmSubsystem get() {
    return a_subsystem;
  }

  /***
   * Configures our motors with the exact same settings
   * 
   * @param motorController
   *                        The CANSparkMax to configure
   */
  public static void configMotorController(CANSparkMax motorController) {
    motorController.restoreFactoryDefaults();
    motorController.setIdleMode(IdleMode.kBrake);
    motorController.enableVoltageCompensation(12);
    motorController.setSmartCurrentLimit(10);
  }

  // reset arm
  public void resetArm() {
    m_armMotorEncoder.setPosition(0);
  }

  /**
   * Makes our drive motors spin at the specified speeds
   * 
   * @param armSpeed
   *   Speed of the front left wheel in duty cycles [-1, 1]
   */
  public void setDriveMotors(double speed) {
    // Ensure speed is within valid range
    speed = Math.min(1, Math.max(speed, -1));
    m_armMotor.set(speed);
    boolean isHit = (speed > 0) ? m_aTopSwitch.get() : m_aBotSwitch.get();
    if (isHit) {
      ((BrakePneumatics) p_subsystem).armBrake();
    } else {
      ((BrakePneumatics) p_subsystem).armInMovement();
    }
  }

 @Override
  public void periodic() {
    m_armMotor.set(.5);
    // m_armMotor.set(m_backRightPIDController.calculate(m_backRightCANCoder.getAbsolutePosition()));
    // add pid controller for arm?? is it needed?
  }
};