package frc.trigon.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.ExecuteEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

import java.util.Set;
import java.util.function.Supplier;

public class SpindexerCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetVelocityDegrees) -> RobotContainer.SPINDEXER.setTargetVelocity(targetVelocityDegrees),
                false,
                Set.of(RobotContainer.SPINDEXER),
                "Debugging/TargetSpindexerVelocity"
        ).finallyDo(
                interrupted -> RobotContainer.SPINDEXER.setTargetVelocity(0)
        );
    }

    public static Command getSetTargetStateCommand(SpindexerConstants.SpindexerState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.SPINDEXER.setTargetState(targetState),
                RobotContainer.SPINDEXER::stop,
                RobotContainer.SPINDEXER
        );
    }

    public static Command getSetTargetVelocityCommand(Supplier<Double> targetVelocity) {
        return new ExecuteEndCommand(
                () -> RobotContainer.SPINDEXER.setTargetVelocity(targetVelocity.get()),
                RobotContainer.SPINDEXER::stop,
                RobotContainer.SPINDEXER
        );
    }
}