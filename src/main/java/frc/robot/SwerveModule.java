// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class SwerveModule {
    private PIDController m_PIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD, DriveConstants.kSteerPeriod);
    private CANCoder m_CANCoder;
    private CANSparkMax m_driveMotor;
    public RelativeEncoder m_driveEncoder;
    private CANSparkMax m_steerMotor;

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
     * 
     * @param motorController);

    }

     *                        The CANSparkMax to configure
     */
    public static void configMotorController(CANSparkMax motorController) {
        motorController.restoreFactoryDefaults();
        motorController.setIdleMode(IdleMode.kBrake);
        motorController.enableVoltageCompensation(12);
        motorController.setSmartCurrentLimit(10);
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
