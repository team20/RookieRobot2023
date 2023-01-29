// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class CounterWeightSubsystem extends SubsystemBase {
  private CANSparkMax m_counterWeightMotor = new CANSparkMax(DriveConstants.kcounterWeightMotor, MotorType.kBrushless);
  public RelativeEncoder m_counterWeightEncoder = m_counterWeightMotor.getEncoder();
  private static CounterWeightSubsystem s_subsystem;
  
  public CounterWeightSubsystem() {
    if (s_subsystem != null) {
      try {
        throw new Exception("Motor subsystem already initalized!");
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

  public static void configMotorController(CANSparkMax motorController) {
    motorController.restoreFactoryDefaults();
    motorController.setIdleMode(IdleMode.kBrake);
    motorController.enableVoltageCompensation(12);
    motorController.setSmartCurrentLimit(10);
  }

  /**
  * Makes our drive motors spin at the specified speeds
  * 
  * @param counterWeightMotorSpeed
  *                        Speed of the front left wheel in duty cycles [-1, 1]
  */

  public void setDriveMotors(double counterWeightMotorSpeed) {
    m_counterWeightMotor.set(counterWeightMotorSpeed);
  }

  @Override
  public void periodic() {
    m_counterWeightMotor.set(.2);
  }
}
