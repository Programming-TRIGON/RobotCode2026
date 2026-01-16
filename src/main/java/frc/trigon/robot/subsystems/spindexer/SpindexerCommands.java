package frc.trigon.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

import java.util.Set;

public class SpindexerCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.SPINDEXER::setTargetVelocity,
                false,
                Set.of(RobotContainer.SPINDEXER),
                "Debugging/SpindexerTargetVelocity"
        );
    }

    public static Command getSetTargetStateCommand(SpindexerConstants.SpindexerState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.SPINDEXER.setTargetState(targetState),
                RobotContainer.SPINDEXER::stop,
                RobotContainer.SPINDEXER
        );
    }

    public static Command getSetTargetVelocityRotationsPerSecondCommand(double targetVelocity) {
        return new StartEndCommand(
                () -> RobotContainer.SPINDEXER.setTargetVelocity(targetVelocity),
                RobotContainer.SPINDEXER::stop,
                RobotContainer.SPINDEXER
        );
    }
}