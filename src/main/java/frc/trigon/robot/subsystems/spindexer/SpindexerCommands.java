package frc.trigon.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

import java.util.Set;

public class SpindexerCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.SPINDEXER::setTargetVelocityRotationsPerSecond,
                false,
                Set.of(RobotContainer.SPINDEXER),
                "Debugging/SpindexerTargetVelocityRotationsPerSecond"
        );
    }

    public static Command getSetTargetStateCommand(SpindexerConstants.SpindexerState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.SPINDEXER.setTargetState(targetState),
                RobotContainer.SPINDEXER::stop,
                RobotContainer.SPINDEXER
        );
    }

    public static Command getSetTargetVelocityCommand(double targetVelocityRotationsPerSecond) {
        return new StartEndCommand(
                () -> RobotContainer.SPINDEXER.setTargetVelocityRotationsPerSecond(targetVelocityRotationsPerSecond),
                RobotContainer.SPINDEXER::stop,
                RobotContainer.SPINDEXER
        );
    }
}