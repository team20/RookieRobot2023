// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

public class DriveSubsystem extends SubsystemBase {
  private SwerveModule m_frontLeftSwerveModule;
  private SwerveModule m_frontRightSwerveModule;
  private SwerveModule m_backLeftSwerveModule;
  private SwerveModule m_backRightSwerveModule;
  private static DriveSubsystem s_subsystem;


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Singleton
    if (s_subsystem != null) {
      try {
        throw new Exception("Motor subsystem already initalized!");
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
    s_subsystem = this;

    // Initialize modules
    {
      m_frontLeftSwerveModule = new SwerveModule(
        DriveConstants.kFrontLeftCANCoderPort, 
        DriveConstants.kFrontLeftDrivePort, 
        DriveConstants.kFrontLeftSteerPort, 
        SwerveConstants.FrontLeftZero, 
        DriveConstants.kFrontLeftDriveInverted
        );

      m_frontRightSwerveModule = new SwerveModule(
        DriveConstants.kFrontRightCANCoderPort, 
        DriveConstants.kFrontRightDrivePort, 
        DriveConstants.kFrontRightSteerPort, 
        SwerveConstants.FrontRightZero, 
        DriveConstants.kFrontRightDriveInverted);

      m_backLeftSwerveModule = new SwerveModule(
        DriveConstants.kBackLeftCANCoderPort, 
        DriveConstants.kBackLeftDrivePort, 
        DriveConstants.kBackLeftSteerPort, 
        SwerveConstants.BackLeftZero, 
        DriveConstants.kBackLeftDriveInverted);

      m_backRightSwerveModule = new SwerveModule(
        DriveConstants.kBackRightCANCoderPort, 
        DriveConstants.kBackRightDrivePort, 
        DriveConstants.kBackRightSteerPort, 
        SwerveConstants.BackRightZero, 
        DriveConstants.kBackRightDriveInverted);
    }
    getFrontLeftSwerveModule().getDriveEncoder().setPosition(0);
    getFrontRightSwerveModule().getDriveEncoder().setPosition(0);
    getBackLeftSwerveModule().getDriveEncoder().setPosition(0);
    getBackRightSwerveModule().getDriveEncoder().setPosition(0);
  }

  public static DriveSubsystem get() {
    return s_subsystem;
  }

 

  public double getHeading() {
    return 0;
  }

  public void setWheelRotationToZeroDegrees() {
    setSteerMotors(0, 0, 0, 0);
  }

  /**
   * Makes our drive motors spin at the specified speeds
   * 
   * @param frontLeftSpeed
   *                        Speed of the front left wheel in duty cycles [-1, 1]
   * @param frontRightSpeed
   *                        Speed of the front right wheel in duty cycles [-1, 1]
   * @param backLeftSpeed
   *                        Speed of the back left wheel in duty cycles [-1, 1]
   * @param backRightSpeed
   *                        Speed of the back right wheel in duty cycles [-1, 1]
   */
  public void setDriveMotors(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed,
      double backRightSpeed) {
    m_frontLeftSwerveModule.getDriveMotor().set(frontLeftSpeed);
    m_frontRightSwerveModule.getDriveMotor().set(frontRightSpeed);
    m_backLeftSwerveModule.getDriveMotor().set(backLeftSpeed);
    m_backRightSwerveModule.getDriveMotor().set(backRightSpeed);
  }

  /***
   * Sets the target angles in degrees for each wheel on the robot
   * 
   * @param frontLeftAngle
   *                        The target angle of the front left wheel in degrees
   * @param frontRightAngle
   *                        The target angle of the front right wheel in degrees
   * @param backLeftAngle
   *                        The target angle of the back left wheel in degrees
   * @param backRightAngle
   *                        The target angle of the back right wheel in degrees
   */
  public void setSteerMotors(double frontLeftAngle, double frontRightAngle, double backLeftAngle,
      double backRightAngle) {
    m_frontLeftSwerveModule.getPIDController().setSetpoint(frontLeftAngle);
    m_frontRightSwerveModule.getPIDController().setSetpoint(frontRightAngle);
    m_backLeftSwerveModule.getPIDController().setSetpoint(backLeftAngle);
    m_backRightSwerveModule.getPIDController().setSetpoint(backRightAngle);
  }

  /***
   * Recalculates the PID output, and uses it to drive our steer motors. Also logs
   * wheel rotations, and PID setpoints
   */
  @Override
  public void periodic() {
    System.out.println("running"); 
    // For each of our steer motors, feed the current angle of the wheel into its
    // PID controller, and use it to calculate the duty cycle for its motor, and
    // spin the motor
    m_frontLeftSwerveModule.getSteerMotor().set(m_frontLeftSwerveModule.getPIDController() .calculate(m_frontLeftSwerveModule.getCANCoder().getAbsolutePosition()));
    m_frontRightSwerveModule.getSteerMotor().set(m_frontRightSwerveModule.getPIDController().calculate(m_frontRightSwerveModule.getCANCoder().getAbsolutePosition()));
    m_backLeftSwerveModule.getSteerMotor().set(m_backLeftSwerveModule.getPIDController()  .calculate(m_backLeftSwerveModule.getCANCoder().getAbsolutePosition()));
    m_backRightSwerveModule.getSteerMotor().set(m_backRightSwerveModule.getPIDController() .calculate(m_backRightSwerveModule.getCANCoder().getAbsolutePosition()));
    // Logging
    SmartDashboard.putNumber("FL CANCoder Rotation", m_frontLeftSwerveModule.getCANCoder().getAbsolutePosition());
    SmartDashboard.putNumber("FR CANCoder Rotation", m_frontRightSwerveModule.getCANCoder().getAbsolutePosition());
    SmartDashboard.putNumber("BL CANCoder Rotation", m_backLeftSwerveModule.getCANCoder().getAbsolutePosition());
    SmartDashboard.putNumber("BR CANCoder Rotation", m_backRightSwerveModule.getCANCoder().getAbsolutePosition());
    SmartDashboard.putNumber("FL PID Setpoint", m_frontLeftSwerveModule.getPIDController().getSetpoint());
    SmartDashboard.putNumber("FR PID Setpoint", m_frontRightSwerveModule.getPIDController().getSetpoint());
    SmartDashboard.putNumber("BL PID Setpoint", m_backLeftSwerveModule.getPIDController().getSetpoint());
    SmartDashboard.putNumber("BR PID Setpoint", m_backRightSwerveModule.getPIDController().getSetpoint());
    
    double m_flEncoderPos = m_frontLeftSwerveModule.getDriveEncoderPosition();
    double m_frEncoderPos = m_frontRightSwerveModule.getDriveEncoderPosition();
    double m_blEncoderPos = m_backLeftSwerveModule.getDriveEncoderPosition();
    double m_brEncoderPos = m_backRightSwerveModule.getDriveEncoderPosition();
    
    SmartDashboard.putNumber("FL DriveEncoder", m_flEncoderPos);
    SmartDashboard.putNumber("FR DriveEncoder", m_frEncoderPos); 
    SmartDashboard.putNumber("BL DriveEncoder", m_blEncoderPos); 
    SmartDashboard.putNumber("BR DriveEncoder", m_brEncoderPos); 
    
  }


  public SwerveModule getFrontLeftSwerveModule() {
    return this.m_frontLeftSwerveModule;
  }

  public SwerveModule getFrontRightSwerveModule() {
    return this.m_frontRightSwerveModule;
  }

  public SwerveModule getBackLeftSwerveModule() {
    return this.m_backLeftSwerveModule;
  }

  public SwerveModule getBackRightSwerveModule() {
    return this.m_backRightSwerveModule;
  }
}
