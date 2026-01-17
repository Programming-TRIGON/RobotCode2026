package frc.trigon.robot.subsystems.transporter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

import java.util.Set;

public class TransporterCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.TRANSPORTER::setTargetVelocityMetersPerSecond,
                false,
                Set.of(RobotContainer.TRANSPORTER),
                "Debugging/TransporterTargetVelocity"
        );
    }

    public static Command getSetTargetStateCommand(TransporterConstants.TransporterState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.TRANSPORTER.setTargetState(targetState),
                RobotContainer.TRANSPORTER::stop,
                RobotContainer.TRANSPORTER
        );
    }

    public static Command getSetTargetVelocity(double targetVelocityMetersPerSecond) {
        return new StartEndCommand(
                () -> RobotContainer.TRANSPORTER.setTargetVelocityMetersPerSecond(targetVelocityMetersPerSecond),
                RobotContainer.TRANSPORTER::stop,
                RobotContainer.TRANSPORTER
        );
    }

}
