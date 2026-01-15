package frc.trigon.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.ExecuteEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

import java.util.Set;

public class SpindexerCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetVelocity) -> RobotContainer.SPINDEXER.setTargetVelocity(targetVelocity),
                false,
                Set.of(RobotContainer.SPINDEXER),
                "Debugging/TargetSpindexerVelocity"
        );
    }

    public static Command getSetTargetStateCommand(SpindexerConstants.SpindexerState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.SPINDEXER.setTargetState(targetState),
                RobotContainer.SPINDEXER::stop,
                RobotContainer.SPINDEXER
        );
    }

    public static Command getSetTargetVelocityCommand(double targetVelocity) {
        return new ExecuteEndCommand(
                () -> RobotContainer.SPINDEXER.setTargetVelocity(targetVelocity),
                RobotContainer.SPINDEXER::stop,
                RobotContainer.SPINDEXER
        );
    }
}