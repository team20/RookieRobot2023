package frc.robot;

import javax.management.RuntimeErrorException;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoDriveCommand;

public class CommandFactory {

    public static enum AutoType {
        //TODO expand possibilities
        LEAVE_AREA, SCORE_THEN_LEAVE, SCORE_THEN_DOCK
    }

    public static SequentialCommandGroup createAuto(AutoType autoType){
        //TODO implement auto here
        switch(autoType){
            case LEAVE_AREA:
                return new SequentialCommandGroup(new AutoDriveCommand(0.0, -0.25, 5, 0.08));
            default:
                System.err.println("Using unimplemented auto!: " + autoType.toString() + (0 / 0));
        }
        return null;
    }
}
