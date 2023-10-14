package frc.robot.commands.pneumatic;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotPneumaticsSubsystem;

public class PivotPneumaticCommand extends CommandBase {
	public enum Operation { RAISE, LOWER, TOGGLE };      
	protected PivotPneumaticsSubsystem m_pivot;
	protected Operation m_op;

	public PivotPneumaticCommand(PivotPneumaticsSubsystem pivot, Operation operation) {
		m_pivot = pivot;
		addRequirements(pivot);
		this.m_op = operation;
	}

	@Override
	public void initialize() {
		switch(m_op){
            case RAISE:  m_pivot.setRaise();   break;
            case LOWER:  m_pivot.setLower();   break;
			case TOGGLE: m_pivot.setToggle();  break;
        }
	}

	/**
	 * Performs the action needed when this {@code ControlIntakePneumaticCommand} is finished/interrupted.
	 */
	@Override
	public void end(boolean interrupted) {
		if (m_op == Operation.TOGGLE)
			m_pivot.setRaise();
	}

	/**
	 * Determines whether or not this {@code ControlIntakePneumaticCommand} is finished.
	 */
	@Override
	public boolean isFinished() {
		return (m_op == Operation.TOGGLE) ? false : true;
	}
}