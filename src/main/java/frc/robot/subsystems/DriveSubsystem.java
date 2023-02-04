// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

public class DriveSubsystem extends SubsystemBase {
  private SwerveModule m_frontLeftSwerveModule;
  private SwerveModule m_frontRightSwerveModule;
  private SwerveModule m_backLeftSwerveModule;
  private SwerveModule m_backRightSwerveModule;
  private SwerveDriveOdometry m_odometry;
  private static DriveSubsystem s_subsystem;
  private AHRS m_NAVX = new AHRS(SPI.Port.kMXP);


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

    

    m_odometry = new SwerveDriveOdometry(DriveConstants.kSwerveKinematics, new Rotation2d(0), getSwervePositions());
    m_frontLeftSwerveModule = new SwerveModule(
      DriveConstants.kFrontLeftCANCoderPort, 
      DriveConstants.kFrontLeftDrivePort, 
      DriveConstants.kFrontLeftSteerPort, 
      SwerveConstants.kFrontLeftZero, 
      DriveConstants.kFrontLeftDriveInverted
      );

    m_frontRightSwerveModule = new SwerveModule(
      DriveConstants.kFrontRightCANCoderPort, 
      DriveConstants.kFrontRightDrivePort, 
      DriveConstants.kFrontRightSteerPort, 
      SwerveConstants.kFrontRightZero, 
      DriveConstants.kFrontRightDriveInverted);

    m_backLeftSwerveModule = new SwerveModule(
      DriveConstants.kBackLeftCANCoderPort, 
      DriveConstants.kBackLeftDrivePort, 
      DriveConstants.kBackLeftSteerPort, 
      SwerveConstants.kBackLeftZero, 
      DriveConstants.kBackLeftDriveInverted);

    m_backRightSwerveModule = new SwerveModule(
      DriveConstants.kBackRightCANCoderPort, 
      DriveConstants.kBackRightDrivePort, 
      DriveConstants.kBackRightSteerPort, 
      SwerveConstants.kBackRightZero, 
      DriveConstants.kBackRightDriveInverted);

    //zero Navx
    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      }catch(Exception ex){}
    });
    resetEncoders();
  }

  public static DriveSubsystem get() {
    return s_subsystem;
  }

  public void resetEncoders() {
    // Zero drive encoders
    m_frontLeftSwerveModule.getDriveEncoder().setPosition(0);
    m_frontRightSwerveModule.getDriveEncoder().setPosition(0);
    m_backLeftSwerveModule.getDriveEncoder().setPosition(0);
    m_backRightSwerveModule.getDriveEncoder().setPosition(0);
  }

  public void zeroHeading(){
    m_NAVX.reset();
  }

  public double getHeading() {
    // Return the angle from -180, 180
    return Math.IEEEremainder(m_NAVX.getAngle(), 360);
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

  /**
   * Makes our drive motors spin based on given module states
   * 
   * @param states
   *        The wanted states of [FL, FR, BL, BR] swerve modules
   */
  public void setDriveMotors(SwerveModuleState[] states) {
    m_frontLeftSwerveModule.getDriveMotor().set(states[0].speedMetersPerSecond / Constants.DriveConstants.kMaxVelocity);
    m_frontRightSwerveModule.getDriveMotor().set(states[1].speedMetersPerSecond / Constants.DriveConstants.kMaxVelocity);
    m_backLeftSwerveModule.getDriveMotor().set(states[2].speedMetersPerSecond / Constants.DriveConstants.kMaxVelocity);
    m_backRightSwerveModule.getDriveMotor().set(states[3].speedMetersPerSecond / Constants.DriveConstants.kMaxVelocity);
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
    // System.out.println("running"); 
    m_odometry.update(getRobotPose2D().getRotation(), getSwervePositions());

    // For each of our steer motors, feed the current angle of the wheel into its
    // PID controller, and use it to calculate the duty cycle for its motor, and
    // spin the motor

    m_frontLeftSwerveModule.getSteerMotor().set(m_frontLeftSwerveModule.getPIDController().calculate(m_frontLeftSwerveModule.getCANCoder().getAbsolutePosition()));
    m_frontRightSwerveModule.getSteerMotor().set(m_frontRightSwerveModule.getPIDController().calculate(m_frontRightSwerveModule.getCANCoder().getAbsolutePosition()));
    m_backLeftSwerveModule.getSteerMotor().set(m_backLeftSwerveModule.getPIDController().calculate(m_backLeftSwerveModule.getCANCoder().getAbsolutePosition()));
    m_backRightSwerveModule.getSteerMotor().set(m_backRightSwerveModule.getPIDController().calculate(m_backRightSwerveModule.getCANCoder().getAbsolutePosition()));
    
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

  /**
   * Returns the position and rotation of the robot in a 2D space
   * for auto trajectory following
   * @return the robot's pose
   * @see Pose2d
   */
  public Pose2d getRobotPose2D(){
    return m_odometry.getPoseMeters();
  }

  public SwerveModulePosition[] getSwervePositions(){
    SwerveModulePosition[] poses = { 
      m_frontLeftSwerveModule.getPosition(), 
      m_frontRightSwerveModule.getPosition(), 
      m_backLeftSwerveModule.getPosition(), 
      m_backRightSwerveModule.getPosition()
    };
    return poses;  
  }

  public void setOdometryPosition(Pose2d pos){
    m_odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), getSwervePositions(), pos);
  }
}
