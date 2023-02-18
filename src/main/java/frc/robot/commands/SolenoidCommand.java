package frc.robot.commands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SolenoidCommand extends CommandBase {
    private DoubleSolenoid m_solenoid;
    // private Compressor m_compressor;

    public SolenoidCommand(int solPort1, int solPort2) {
            //  = new Compressor(1, PneumaticsModuleType.REVPH);
            m_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, solPort1, solPort2);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //m_solenoid.setPulseDuration(.50);
        //m_solenoid.startPulse();
        m_solenoid.set(DoubleSolenoid.Value.kForward);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void cancel() {
    }
}