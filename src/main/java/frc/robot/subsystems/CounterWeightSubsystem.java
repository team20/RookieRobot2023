// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.DriveConstants;


public class CounterWeightSubsystem extends SubsystemBase {
  private CANSparkMax m_counterWeightMotor = new CANSparkMax(DriveConstants.kCounterWeightPort, MotorType.kBrushless);
  public RelativeEncoder m_counterWeightMotorEncoder = m_counterWeightMotor.getEncoder();
  private static CounterWeightSubsystem s_subsystem;
  DigitalInput toplimitSwitch = new DigitalInput(0);
  DigitalInput bottomlimitSwitch = new DigitalInput(1);
  
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
  }

  /**
   * Makes our drive motors spin at the specified speeds
   * 
   * @param counterWeightSpeed
   *                        Speed of the front left wheel in duty cycles [-1, 1]
   */
  public void setDriveMotors(double speed) {
    if (speed > 0) {
      if (toplimitSwitch.get()) {
          // We are going up and top limit is tripped so stop
          m_counterWeightMotor.set(0);
      } else {
          // We are going up but top limit is not tripped so go at commanded speed
          m_counterWeightMotor.set(speed);
      }
    } else {
      if (bottomlimitSwitch.get()) {
          // We are going down and bottom limit is tripped so stop
          m_counterWeightMotor.set(0);
      } else {
          // We are going down but bottom limit is not tripped so go at commanded speed
          m_counterWeightMotor.set(speed);
      }
    }
  }
  // Counter Weight Movement Algorithm
  
  public REVLibError counterWeightAlgorithm(RelativeEncoder m_armMotorEncoder){
    //Establishes ratio between motor encoders (ie. 5 rotations in arm = 1 rotation in counterweight)
    double armToCounterWeightEncoderRatio = 0.2;
    //Always checking for if statement condition which tells the counterweight encoder what position to move to
    while (true){
      double armMotorEncoderPosition = m_armMotorEncoder.getPosition();
      double counterWeightMotorEncoderInitialPosition = m_counterWeightMotorEncoder.getPosition();
        if ((armMotorEncoderPosition-counterWeightMotorEncoderInitialPosition)>5){
            continue;
          }
        REVLibError counterWeightMotorEncoderFinalPosition = m_counterWeightMotorEncoder.setPosition(armMotorEncoderPosition*armToCounterWeightEncoderRatio);
        return counterWeightMotorEncoderFinalPosition;
      } 
    }

  @Override
  public void periodic() {
    m_counterWeightMotor.set(.5);
  }
}