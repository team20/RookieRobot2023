// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CommandFactory.AutoType;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.Constants.ControllerConstants.DPad;
import frc.robot.commands.arm.DefaultArmCommand;
import frc.robot.commands.counterweight.DefaultCounterWeightCommand;
import frc.robot.commands.drive.AutoDriveCommand;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.TurnWheelsToZeroDegreesCommand;
import frc.robot.commands.pneumatic.ClawPneumaticCommand;
import frc.robot.commands.pneumatic.PivotPneumaticCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CounterWeightSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem.ClawPneumatics;
import frc.robot.subsystems.PneumaticsSubsystem.PivotPneumatics;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Joystick m_djoystick1 = new Joystick(ControllerConstants.kDriverControllerPort);
  private final Joystick m_ojoystick2 = new Joystick(ControllerConstants.kOperatorControllerPort);
  private final GenericHID m_dcontroller = new GenericHID(ControllerConstants.kDriverControllerPort);
  private final GenericHID m_ocontroller = new GenericHID(ControllerConstants.kOperatorControllerPort);
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final CounterWeightSubsystem m_counterWeightSubsystem = new CounterWeightSubsystem();
  private final ClawPneumatics m_claw = new ClawPneumatics();
  private final PivotPneumatics m_pivot = new PivotPneumatics();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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
          () -> m_djoystick1.getRawAxis(Axis.kLeftX),
          () -> m_djoystick1.getRawAxis(Axis.kLeftY),
          () -> m_djoystick1.getRawAxis(Axis.kRightX)));

    m_armSubsystem.setDefaultCommand(
      new DefaultArmCommand(
          m_armSubsystem,
          () -> m_ojoystick2.getRawAxis(Axis.kRightY)));

          
    m_counterWeightSubsystem.setDefaultCommand(
      new DefaultCounterWeightCommand(
        m_counterWeightSubsystem,
          () -> m_ojoystick2.getRawAxis(Axis.kLeftX)));


    // Register button(s) to raise (pivot) the claw
    new POVButton(m_ocontroller, DPad.kUp).or(new POVButton(m_ocontroller, DPad.kUpLeft))
      .or(new POVButton(m_ocontroller, DPad.kUpRight))
      .onTrue(new PivotPneumaticCommand(m_pivot, PivotPneumaticCommand.Operation.RAISE));

    // Register button(s) to lower (pivot) the claw
    new POVButton(m_ocontroller, DPad.kDown).or(new POVButton(m_ocontroller, DPad.kDownLeft))
      .or(new POVButton(m_ocontroller, DPad.kDownRight))
      .onTrue(new PivotPneumaticCommand(m_pivot, PivotPneumaticCommand.Operation.LOWER));

    // Register button to open the claw
    new JoystickButton(m_ocontroller, Button.kLeftBumper)
      .onTrue(new ClawPneumaticCommand(m_claw, ClawPneumaticCommand.Operation.OPEN));

    // Register button to close the claw
    new JoystickButton(m_ocontroller, Button.kRightBumper)
      .onTrue(new ClawPneumaticCommand(m_claw, ClawPneumaticCommand.Operation.CLOSE));

    new Trigger(() -> m_dcontroller.getRawButton(ControllerConstants.Button.kTriangle))
        .onTrue(new TurnWheelsToZeroDegreesCommand());

    //Register command to turn field centric mode OFF
    new Trigger(() -> m_dcontroller.getRawButton(ControllerConstants.Button.kCircle))
        .onTrue(new InstantCommand((()->{DriveSubsystem.get().setFieldCentric(false);}), DriveSubsystem.get() ));

    //Register command to turn field centric mode ON
    new Trigger(() -> m_dcontroller.getRawButton(ControllerConstants.Button.kSquare))
        .onTrue(new InstantCommand((()->{DriveSubsystem.get().setFieldCentric(true);}), DriveSubsystem.get() ));
    }

  public Command getAutonomousCommand() {
    return CommandFactory.createAuto(AutoType.LEAVE_AREA);
  }
}