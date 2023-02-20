package frc.robot.commands;

import frc.robot.subsystems.PneumaticSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawPneumaticCommand extends CommandBase {
		public enum Operation {
            LOWER, RAISE, TOGGLE };
            
	protected ClawPneumaticSubsystem m_subsystem;
	protected Operation m_op;

	/**0
	 * Constructs a {@code ControlIntakePneumaticCommand}.
	 *
	 * @param subsystem
	 *            the subsystem controlled by this {@code ControlIntakePneumaticCommand}
	 * @param operation
	 *            the operation to carry out
	 */
	public ControlClaePneumaticCommand(IntakePneumaticSubsystem subsystem, Operation operation) {
		this.m_subsystem = subsystem;
		addRequirements(subsystem);
		this.m_op = operation;
	}

	/**
	 * Performs the action needed when this {@code ControlIntakePneumaticCommand} is scheduled for the first time.
	 */
	@Override
	public void initialize() {
		if (operation == Operation.RAISE) {
			subsystem.raiseIntake();
		} else {
			subsystem.lowerIntake();
		}
	}

	/**
	 * Performs the action needed when this {@code ControlIntakePneumaticCommand} is finished/interrupted.
	 */
	@Override
	public void end(boolean interrupted) {
		if (operation == Operation.TOGGLE) {
			subsystem.raiseIntake();
		}
	}

	/**
	 * Determines whether or not this {@code ControlIntakePneumaticCommand} is finished.
	 */
	@Override
	public boolean isFinished() {
		if (operation == Operation.TOGGLE)
			return false;
		else
			return true;
	}
}