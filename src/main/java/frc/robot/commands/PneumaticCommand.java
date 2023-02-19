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
       requires(PneumaticSubsystem.m_solenoid);
       System.out.println("hi")

    }

    // Called once when the command executes
    @Override
    public void initialize() {
       Robot.m_shooter.pitchDown();
    }   
}