// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import java.lang.Math;
public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax m_motor = new CANSparkMax(ArmConstants.kCANDICE, MotorType.kBrushless);
  public RelativeEncoder m_armMotorEncoder = m_motor.getEncoder();
  private static ArmSubsystem m_subsystem;
  private static BrakePneumaticsSubsystem m_brake = new BrakePneumaticsSubsystem();
  private SparkMaxLimitSwitch m_forwardLimit;
  private SparkMaxLimitSwitch m_reverseLimit;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // Singleton
    if (m_subsystem != null) {
      try {
        throw new Exception("Arm subsystem already initalized!");
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
    m_subsystem = this;
    configMotorController();
  }

  public static ArmSubsystem get() {
    return m_subsystem;
  }

  
  public void configMotorController() {
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.enableVoltageCompensation(12);
    m_motor.setSmartCurrentLimit(25);                 //arm power limit (out of 60)
    m_forwardLimit = m_motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_reverseLimit = m_motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  }

  // reset arm
  public void resetArm() {
    m_armMotorEncoder.setPosition(0);
  }

  /**
   * Drive the motor at the specified speed
   * 
   * @param speed
   *   Valid Speed  range [-1, 1]
   */
  public void setMotorSpeed(double speed) {
    // Ensure speed is within valid range
    speed = Math.min(1, Math.max(speed, -1));
    m_motor.set(speed);
  }

 @Override
  public void periodic() {
    double speed = m_motor.get();

    if(!(Math.abs(speed) > 0) || m_forwardLimit.isPressed() || m_reverseLimit.isPressed()) {
      new WaitCommand(0.5);
      m_brake.armBrake();  
    } else {
      m_brake.armInMovement();
    }
  
  }
   
};