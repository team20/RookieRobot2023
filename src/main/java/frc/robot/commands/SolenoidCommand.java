package frc.robot.commands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PneumaticSubsystem;

public class SolenoidCommand extends CommandBase {
    private Solenoid m_solenoid;

    public SolenoidCommand() { 
        Solenoid solenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);
        m_solenoid = solenoid;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_solenoid.set(true);
    }

    // @Override
    // public boolean isFinished() {
    // }

    // @Override
    // public void cancel() {
    // }
}