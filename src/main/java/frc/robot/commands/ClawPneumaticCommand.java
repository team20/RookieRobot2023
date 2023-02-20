package frc.robot.commands;
import frc.robot.subsystems.PneumaticsSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem.ClawPneumatics;

public class ClawPneumaticCommand extends CommandBase {
		public enum Operation {
            OPEN, CLOSE, TOGGLE };
            
	protected PneumaticsSubsystem m_subsystem;
	protected Operation m_op;

	/**0
	 * Constructs a {@code ControlIntakePneumaticCommand}.
	 *
	 * @param subsystem
	 *            the subsystem controlled by this {@code ControlIntakePneumaticCommand}
	 * @param operation
	 *            the operation to carry out
	 * @return 
	 */
	public void ControlClawPneumaticCommand(PneumaticsSubsystem subsystem, Operation operation) {
		this.m_subsystem = subsystem;
		addRequirements(subsystem);
		this.m_op = operation;
	}

	/**
	 * Performs the action needed when this {@code ControlIntakePneumaticCommand} is scheduled for the first time.
	 */
	@Override
	public void initialize() {
		switch(m_op){
            case OPEN:
				((ClawPneumatics) m_subsystem).openClaw();
            case CLOSE:
				((ClawPneumatics) m_subsystem).closeClaw();
			case TOGGLE:
				((ClawPneumatics) m_subsystem).toggleClaw();
        }
	}

	/**
	 * Performs the action needed when this {@code ControlIntakePneumaticCommand} is finished/interrupted.
	 */
	@Override
	public void end(boolean interrupted) {
		if (m_op == Operation.TOGGLE) {
			((ClawPneumatics) m_subsystem).openClaw();
		}
	}

	/**
	 * Determines whether or not this {@code ControlIntakePneumaticCommand} is finished.
	 */
	@Override
	public boolean isFinished() {
		if (m_op == Operation.TOGGLE)
			return false;
		else
			return true;
	}
}





