package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoDriveCommand;
import frc.robot.commands.drive.AutoTurnCommand;
import frc.robot.subsystems.DriveSubsystem;

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
                    new AutoDriveCommand(0.0, -0.75, 2, 0.25)
                    );
            case DOCK:
                return new SequentialCommandGroup(
                    new InstantCommand((()->{ DriveSubsystem.get().resetHeading(); })),
                    new AutoDriveCommand(0.0, -1, 4, 0.08),
                    new AutoDriveCommand(0.0, -0.35, 3, 0.08),
                    new AutoTurnCommand(165, 0.15, 15),
                    new AutoDriveCommand(0.0, 1.15, 4, 0.08),
                    new AutoDriveCommand(0.1, 2, 6, 0.08)
                    //new PIDAutoBalanceCommand()
                    );
            default:
                System.err.println("Using unimplemented auto!: " + autoType.toString() + (0 / 0));
        }
        return null;
    }
}
