// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ResetToZeroDegreesCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Joystick m_joystick = new Joystick(ControllerConstants.kDriverControllerPort);
  private final GenericHID m_controller = new GenericHID(ControllerConstants.kDriverControllerPort);
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private Trajectory trajectory;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
      //Read auto trajectory from file
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(Constants.SwerveConstants.kPathTrajectoryFile);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + Constants.SwerveConstants.kPathTrajectoryFile, ex.getStackTrace());
      }

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driveSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            m_driveSubsystem,
            () -> m_joystick.getRawAxis(Axis.kLeftX),
            () -> m_joystick.getRawAxis(Axis.kLeftY),
            () -> m_joystick.getRawAxis(Axis.kRightX)));
    new Trigger(() -> m_controller.getRawButton(ControllerConstants.Button.kTriangle))
        .onTrue(new ResetToZeroDegreesCommand());
  }

  public Command getAutonomousCommand() {
    //Create PID controllers for swerve movement
    PIDController xController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    PIDController yController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    ProfiledPIDController rotationController = new ProfiledPIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD, DriveConstants.kContraints);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    
    // Create a new instance of a SwerveControllerCommand which will execute, 
    // moving the robot along the spline paths until it has reached its destination
    SwerveControllerCommand controllerCommand = new SwerveControllerCommand
    (
    trajectory,                       // The path to follow
    m_driveSubsystem::getRobotPose2D, // The robot's position
    DriveConstants.kSwerveKinematics, // The robot's physical definition
    xController,                      // The PID controller responsible for moving the robot in the X axis
    yController,                      // The PID controller responsible for moving the robot in the Y axis
    rotationController,               // The PID controller responsible rotating the robot
    m_driveSubsystem::setDriveMotors, // The function to call to set the speeds of the individual motors
    m_driveSubsystem                  // requirement
    );

      return new SequentialCommandGroup(
        new InstantCommand(() -> m_driveSubsystem.setOdometryPosition(trajectory.getInitialPose())), //Reset odometry to the starting point of the robot
        controllerCommand,
        new InstantCommand(() -> m_driveSubsystem.setDriveMotors(0, 0, 0, 0))
        );

      /*
    return new SequentialCommandGroup(new CalibrationAutoCommand(CalibrationAutoCommand.Operation.CMD_ANGLE, 0),
                                     new CalibrationAutoCommand(CalibrationAutoCommand.Operation.CMD_DISTANCE, 2),
                                      new CalibrationAutoCommand(CalibrationAutoCommand.Operation.CMD_ANGLE, 90),
                                      new CalibrationAutoCommand(CalibrationAutoCommand.Operation.CMD_DISTANCE, 2),
                                      new CalibrationAutoCommand(CalibrationAutoCommand.Operation.CMD_ANGLE, 180),
                                      new CalibrationAutoCommand(CalibrationAutoCommand.Operation.CMD_DISTANCE, 2),
                                      new CalibrationAutoCommand(CalibrationAutoCommand.Operation.CMD_ANGLE, 270),
                                      new CalibrationAutoCommand(CalibrationAutoCommand.Operation.CMD_DISTANCE, 2));
                                       */
  } 
}
