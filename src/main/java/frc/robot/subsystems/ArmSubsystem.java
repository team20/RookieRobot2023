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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WeightConstants;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem.BrakePneumatics;
import java.lang.Math;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax m_motor = new CANSparkMax(ArmConstants.kCANDICE, MotorType.kBrushless);
  public RelativeEncoder m_armMotorEncoder = m_motor.getEncoder();
  private static ArmSubsystem m_subsystem = null;
  private static BrakePneumatics m_brake = new BrakePneumatics();
  DigitalInput m_aTopSwitch = new DigitalInput(ArmConstants.kArmTopLimitDIO);
  DigitalInput m_aBotSwitch = new DigitalInput(ArmConstants.kArmBotLimitDIO);

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
  }

  public static ArmSubsystem get() {
    return m_subsystem;
  }


  public void configMotorController() {
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.enableVoltageCompensation(12);
    m_motor.setSmartCurrentLimit(10);
  }

  // reset arm
  public void resetArm() {
    m_armMotorEncoder.setPosition(0);
  }

  /**
   * Drive the motor at the specified speed
   * 
   * @param speed
   *   Speed of the front left wheel in duty cycles [-1, 1]
   */
  public void setDriveSpeed(double speed) {
    // Ensure speed is within valid range
    speed = Math.min(1, Math.max(speed, -1));
    m_motor.set(speed);
  }

 @Override
  public void periodic() {
    double speed = m_motor.get();
    if(Math.abs(speed) > 0)
      m_brake.disengage();
    else
      m_brake.engage();
  }
};