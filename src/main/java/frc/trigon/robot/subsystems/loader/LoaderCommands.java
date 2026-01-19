package frc.trigon.robot.subsystems.loader;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

import java.util.Set;

public class LoaderCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.LOADER::setTargetVelocity,
                false,
                Set.of(RobotContainer.LOADER),
                "Debugging/LoaderTargetVelocityMetersPerSecond"
        );
    }

    public static Command getSetTargetStateCommand(LoaderConstants.LoaderState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.LOADER.setTargetState(targetState),
                RobotContainer.LOADER::stop,
                RobotContainer.LOADER
        );
    }

    public static Command getSetTargetVelocityCommand(double targetVelocityMetersPerSecond) {
        return new StartEndCommand(
                () -> RobotContainer.LOADER.setTargetVelocity(targetVelocityMetersPerSecond),
                RobotContainer.LOADER::stop,
                RobotContainer.LOADER
        );
    }

    public static Command getFeedToShooterCommand() {
        return new StartEndCommand(
                () -> RobotContainer.LOADER.feedToShooter(),
                RobotContainer.LOADER::stop,
                RobotContainer.LOADER
        );
    }
    
}
