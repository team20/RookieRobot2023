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
import frc.robot.Constants.WeightConstants;
import java.lang.Math;

public class CounterWeightSubsystem extends SubsystemBase {
  private CANSparkMax m_counterWeightMotor = new CANSparkMax(WeightConstants.kCADID, MotorType.kBrushless);
  public RelativeEncoder m_counterWeightMotorEncoder = m_counterWeightMotor.getEncoder();
  private static CounterWeightSubsystem s_subsystem;
  DigitalInput m_topSwitch = new DigitalInput(WeightConstants.kTopLimitDIO);
  DigitalInput m_botSwitch = new DigitalInput(WeightConstants.kBotLimitDIO);
  
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
   *   Speed of the front left wheel in duty cycles [-1, 1]
   */
  public void setDriveMotors(double speed) {
    // Ensure speed is within valid range
    double speed = Math.min(1, Math.max(speed, -1));
    int isHit = (speed > 0) ? m_topSwitch.get() : m_botSwitch.get();
    m_counterWeightMotor.set(isHit ? 0 : speed);
  }
  
  // Counter Weight Movement Algorithm for arm balancing
  public REVLibError counterWeightForArm(RelativeEncoder m_armMotorEncoder){

    // BELOW VALUES TO BE PUT INTO Constants.Java
    double kArmMotorEncoderPositionZero = 0;
    double kCounterWeightEncoderPositionZero = 0;
    REVLibError armMotorStartingPosition = m_armMotorEncoder.setPosition(kArmMotorEncoderPositionZero);
    REVLibError counterWeightMotorStartingPosition = m_counterWeightMotorEncoder.setPosition(kCounterWeightEncoderPositionZero);
    // ABOVE VALUES TO BE PUT INTO Constants.Java

    //Establishes ratio between motor encoders (ie. 5 rotations in arm = 1 rotation in counterweight)
    double armToCounterWeightEncoderRatio = 0.2;

    //Checking for if statement condition which tells the counterweight encoder what position to move to
    while (true){
      double armMotorEncoderInputPosition1 = m_armMotorEncoder.getPosition();
      if (armMotorEncoderInputPosition1 != kArmMotorEncoderPositionZero){
        continue;
      }
      double armMotorEncoderInputPosition2 = m_armMotorEncoder.getPosition();
      double armMotorInputPositionChange = armMotorEncoderInputPosition1-armMotorEncoderInputPosition2;
      double counterWeightMotorEncoderInitialPosition = m_counterWeightMotorEncoder.getPosition();
      //Checks if the difference between both arm encoder values exceeds 5
        if (armMotorInputPositionChange >5 || armMotorInputPositionChange < -5){
            continue;
          }
        REVLibError counterWeightMotorEncoderFinalPosition = m_counterWeightMotorEncoder.setPosition(counterWeightMotorEncoderInitialPosition + (armMotorInputPositionChange*armToCounterWeightEncoderRatio));
        return counterWeightMotorEncoderFinalPosition;
      }
    }
  // Counter Weight Movement Algorithm for robot balancing & charging dock balancing

 @Override
 public void periodic() {
    m_counterWeightMotor.set(.5);
  }
}