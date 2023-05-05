package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem.ClawPneumatics;

public class ClawPneumaticCommand extends CommandBase {
	public enum Operation { OPEN, CLOSE, TOGGLE };
            
	protected ClawPneumatics m_claw;
	protected Operation m_op;

	public ClawPneumaticCommand(ClawPneumatics claw, Operation operation) {
		this.m_claw = claw;
		addRequirements(claw);
		this.m_op = operation;
	}

	@Override
	public void initialize() {
		System.out.println("Claw initialize");
		switch(m_op) {
            case OPEN:   m_claw.openClaw();    break;
            case CLOSE:  m_claw.closeClaw();   break;
			case TOGGLE: m_claw.toggleClaw();  break;
        }
	}

	/**
	 * Performs the action needed when this {@code ControlIntakePneumaticCommand} is finished/interrupted.
	 */
	@Override
	public void end(boolean interrupted) {
		if (m_op == Operation.TOGGLE) {
			m_claw.openClaw();
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