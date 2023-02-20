package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticSubsystem;

/**
* Add your docs here.
*/
public class PneumaticCommand extends CommandBase {
    /**
    * Add your docs here.
    */
    public PneumaticCommand() {
       super();
       // Use requires() here to declare subsystem dependencies
       // eg. requires(chassis);
<<<<<<< HEAD
       requires(PneumaticSubsystem.m_solonoid);
=======
       requires(PneumaticSubsystem.m_solenoid);
>>>>>>> 64b48565e943e8c73ca5b9dd5117ad5dc73938ca

    }

    // Called once when the command executes
    @Override
    public void initialize() {
       Robot.m_shooter.pitchDown();
    }   
}