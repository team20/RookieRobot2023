// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

/**
 * This class represents a single swerve
 * module on a robot, as well as providing
 * a few useful methods for driving and
 * checking encoder values.
 */
public class SwerveModule {
    private PIDController m_PIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD, DriveConstants.kSteerPeriod);
    private CANCoder m_CANCoder;
    private CANSparkMax m_driveMotor;
    public RelativeEncoder m_driveEncoder;
    private CANSparkMax m_steerMotor;


    /**
     * Creates a new instance of the SwerveModule class
     * @param CANport the port of the absolute steering encoder
     * @param drivePort the port of the drive motor
     * @param steerPort the port of the steering motor
     * @param magnetOfset the steering encoder ofset
     * @param inverted is this drive motor inverted?
     */
    public SwerveModule(int CANport, int drivePort, int steerPort, double magnetOfset, boolean inverted){
        m_CANCoder = new CANCoder(CANport);
        m_driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
        m_steerMotor = new CANSparkMax(steerPort, MotorType.kBrushless);
        m_driveEncoder = m_driveMotor.getEncoder();
        m_CANCoder.configMagnetOffset(-magnetOfset);
        configMotorController(m_driveMotor);
        m_driveMotor.setInverted(inverted);
        configMotorController(m_steerMotor);
        m_PIDController.enableContinuousInput(0, 360);
        m_driveEncoder.setPositionConversionFactor(1/SwerveConstants.kMotorRevsPerMeter);
    }
    /***
     * Configures our motors with the exact same settings
     * @param motorController The CANSparkMax to configure
     */
    public static void configMotorController(CANSparkMax motorController) {
        motorController.restoreFactoryDefaults();
        motorController.setIdleMode(IdleMode.kBrake);
        motorController.enableVoltageCompensation(12);
        motorController.setSmartCurrentLimit(30);
    }

    /**
     * Sets this modules steering rotation and drive
     * speed to those specified in a {@code SwerveModuleState}
     * @param state The state to attempt to set
     */
    public void setModuleState(SwerveModuleState state){
        // Makes it so the most a module should ever need
        // to rotate is 90 deg
        // state = SwerveModuleState.optimize(state, new Rotation2d(m_CANCoder.getPosition()));
        // Set drive speed
        m_driveMotor.set(state.speedMetersPerSecond * DriveConstants.kDriveScale);
        m_PIDController.setSetpoint(state.angle.getDegrees());
        //Print state to dashboard
        SmartDashboard.putString("Swerve module " + m_CANCoder.getDeviceID(), state.toString());
    }


    public PIDController getPIDController() {
        return this.m_PIDController;
    }


    public CANCoder getCANCoder() {
        return this.m_CANCoder;
    }


    public CANSparkMax getDriveMotor() {
        return this.m_driveMotor;
    }


    public RelativeEncoder getDriveEncoder() {
        return this.m_driveEncoder;
    }

    public double getDriveEncoderPosition() {
        return this.m_driveEncoder.getPosition();
    }

    public CANSparkMax getSteerMotor() {
        return this.m_steerMotor;
    }


}
