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
//import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.WeightConstants;
import java.lang.Math;

public class CounterWeightSubsystem extends SubsystemBase {
  private CANSparkMax m_counterWeightMotor = new CANSparkMax(WeightConstants.kCANID, MotorType.kBrushless);
  public RelativeEncoder m_counterWeightMotorEncoder = m_counterWeightMotor.getEncoder();
  private static CounterWeightSubsystem s_subsystem;
  //private static SparkMaxLimitSwitch m_forwardLimit;
  //private static SparkMaxLimitSwitch m_reverseLimit;
  
  /** Creates a new CounterWeightSubsystem. */
  public CounterWeightSubsystem() {
    // Singleton
    if (s_subsystem != null) {
      try {
        throw new Exception("Counterweight subsystem already initalized!");
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
    s_subsystem = this;

     // Initialize motors, PID controllers
     {
      configMotorController(m_counterWeightMotor);
    }
  }

  public static CounterWeightSubsystem get() {
    return s_subsystem;
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
   // m_forwardLimit = motorController.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    //m_reverseLimit = motorController.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  }

  /**
   * Makes our drive motors spin at the specified speeds
   * 
   * @param speed valid range [-1, 1]
   */
  public void setDriveMotors(double speed) {
    // Ensure speed is within valid range
    speed = Math.min(1, Math.max(speed, -1));
    //boolean isHit = (speed > 0) ? m_forwardLimit.isPressed() : m_reverseLimit.isPressed();
    m_counterWeightMotor.set(speed);
  }

 @Override
 public void periodic() {
    m_counterWeightMotor.set(.5);
  }
}