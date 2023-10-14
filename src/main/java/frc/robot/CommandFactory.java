package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoDriveCommand;
import frc.robot.commands.drive.PIDAutoBalanceCommand;

public class CommandFactory {

    public static enum AutoType {
        //TODO expand possibilities
        LEAVE_AREA, SCORE_THEN_LEAVE, SCORE_THEN_DOCK, DOCK
    }

    public static SequentialCommandGroup createAuto(AutoType autoType){
        //TODO implement auto here
        switch(autoType){
            case LEAVE_AREA:
                return new SequentialCommandGroup(
                    new AutoDriveCommand(0.0, -0.25, 5, 0.08)
                    );
            case DOCK:
            return new SequentialCommandGroup(
                    new AutoDriveCommand(0.0, -0.25, 3, 0.08),
                    new PIDAutoBalanceCommand()
                    );
            default:
                System.err.println("Using unimplemented auto!: " + autoType.toString() + (0 / 0));
        }
        return null;
    }
}
