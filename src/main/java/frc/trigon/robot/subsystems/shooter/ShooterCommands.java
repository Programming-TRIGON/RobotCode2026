package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.ExecuteEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

import java.util.Set;
import java.util.function.DoubleSupplier;

public class ShooterCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.SHOOTER::setTargetVelocity,
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

    public static Command getAimForDeliveryCommand() {
        return new ExecuteEndCommand(
                RobotContainer.SHOOTER::aimForDelivery,
                RobotContainer.SHOOTER::stop,
                RobotContainer.SHOOTER
        );
    }

    public static Command getAimForEjectionCommand() {
        return new StartEndCommand(
                RobotContainer.SHOOTER::aimForEjection,
                RobotContainer.SHOOTER::stop,
                RobotContainer.SHOOTER
        );
    }

    public static Command getSetTargetVelocityCommand(DoubleSupplier targetVelocityMetersPerSecond) {
        return new StartEndCommand(
                () -> RobotContainer.SHOOTER.setTargetVelocity(targetVelocityMetersPerSecond.getAsDouble()),
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