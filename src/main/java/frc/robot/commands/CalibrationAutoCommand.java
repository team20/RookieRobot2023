package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class CalibrationAutoCommand extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;

    public static enum Operation {
        CMD_ANGLE, CMD_DISTANCE
    }

    Operation m_op;
    double m_amount = 0; // if distance, in ticks; if angle, in degrees
  /***
   * Autonomous command to steer and drive
   * 
   * @param op
   *                        Enumerated mode-steer (CMD_ANGLE) or drive (CMD_DISTANCE)
   * @param amount
   *                        op == CMD_ANGLE: amount is angle in degrees,
   *                        op == CMD_DISTANCE: amount is distance in meters
   * @see Operation
   */
    public CalibrationAutoCommand(Operation op, double amount) { 
        m_driveSubsystem = DriveSubsystem.get();
        m_op = op;
        if (m_op == Operation.CMD_DISTANCE) {
            m_amount = amount; 
        } else if (m_op == Operation.CMD_ANGLE) { 
            m_amount = amount;
        } else {
            try {
                throw new Exception("Unsupported command");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        addRequirements(DriveSubsystem.get());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        System.out.println(m_op == Operation.CMD_ANGLE);
        if (m_op == Operation.CMD_ANGLE) { 
            m_driveSubsystem.setSteerMotors(m_amount, m_amount, m_amount, m_amount);
        } else {      
            // turn on motors - set 20% power for now
            double m_powerLevel = 0.2;
            m_driveSubsystem.setDriveMotors(m_powerLevel, m_powerLevel, m_powerLevel, m_powerLevel);
        }
    }

    @Override
    public boolean isFinished() {
        switch(m_op){
            case CMD_ANGLE:
                //The error between the actual angle and the target angle
                double error = m_driveSubsystem.getFrontLeftSwerveModule().getCANCoder().getPosition() - m_amount;
                return (Math.abs(error) < 2);
            case CMD_DISTANCE:
                //Determine whether the target distance has been reached
                return (m_driveSubsystem.getFrontLeftSwerveModule().getDriveEncoder().getPosition() >= m_amount);
        }
                
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        switch(m_op){
            case CMD_ANGLE:
                break;
            case CMD_DISTANCE:
                //Determine whether the target distance has been reached
                m_driveSubsystem.setDriveMotors(0,0,0,0);
        }
    }
}