package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.ExecuteEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

import java.util.Set;

public class ShooterCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                ShooterCommands::getSetTargetVelocityCommand,
                false,
                Set.of(RobotContainer.SHOOTER),
                "Debugging/TargetShooterVelocityMetersPerSecond"
        );
    }

    public static Command getAimAtHubCommand() {
        return new ExecuteEndCommand(
                RobotContainer.SHOOTER::aimAtHub,
                RobotContainer.SHOOTER::stop,
                RobotContainer.SHOOTER
        );
    }

    public static Command getSetTargetVelocityCommand(double targetVelocityMetersPerSecond) {
        return new StartEndCommand(
                () -> RobotContainer.SHOOTER.setTargetVelocity(targetVelocityMetersPerSecond),
                RobotContainer.SHOOTER::stop,
                RobotContainer.SHOOTER
        );
    }

    public static Command getStopCommand() {
        return new StartEndCommand(
                RobotContainer.SHOOTER::stop,
                () -> {
                },
                RobotContainer.SHOOTER
        );
    }
}