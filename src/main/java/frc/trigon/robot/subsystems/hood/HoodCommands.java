package frc.trigon.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.GearRatioCalculationCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

import java.util.Set;

public class HoodCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetAngleDegrees) -> HoodCommands.getSetTargetAngleCommand(Rotation2d.fromDegrees(targetAngleDegrees)),
                false,
                Set.of(RobotContainer.HOOD),
                "Debugging/HoodTargetAngleDegrees"
        );
    }

    public static Command getGearRatioCalculationCommand() {
        return new GearRatioCalculationCommand(
                HoodConstants.MOTOR,
                HoodConstants.ENCODER,
                0.5,
                RobotContainer.HOOD
        );
    }

    public static Command getAimAtHubCommand() {
        return new StartEndCommand(
                RobotContainer.HOOD::aimAtHub,
                RobotContainer.HOOD::stop,
                RobotContainer.HOOD
        );
    }

    public static Command getDeliveryCommand() {
        return new StartEndCommand(
                RobotContainer.HOOD::aimForDelivery,
                RobotContainer.HOOD::stop,
                RobotContainer.HOOD
        );
    }

    public static Command getRestCommand() {
        return new StartEndCommand(
                RobotContainer.HOOD::rest,
                RobotContainer.HOOD::stop,
                RobotContainer.HOOD
        );
    }

    public static Command getSetTargetAngleCommand(Rotation2d targetAngle) {
        return new StartEndCommand(
                () -> RobotContainer.HOOD.setTargetAngle(targetAngle),
                RobotContainer.HOOD::stop,
                RobotContainer.HOOD
        );
    }
}
