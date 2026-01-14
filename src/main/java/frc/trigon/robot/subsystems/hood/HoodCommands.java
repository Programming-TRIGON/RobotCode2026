package frc.trigon.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

import java.util.Set;

public class HoodCommands {
    public static Command getHoodDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetHoodAngleDegrees) -> {
                    RobotContainer.HOOD.setTargetAngle(Rotation2d.fromDegrees(targetHoodAngleDegrees));
                },
                true,
                Set.of(RobotContainer.HOOD),
                "Debugging/HoodTargetPositionDegrees"
        );
    }

    public static Command getSetTargetAngleCommand(Rotation2d targetAngle) {
        return new StartEndCommand(
                () -> RobotContainer.HOOD.setTargetAngle(targetAngle),
                RobotContainer.HOOD::stop
        );
    }
}
