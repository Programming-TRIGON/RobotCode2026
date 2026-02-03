package frc.trigon.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.lib.commands.ExecuteEndCommand;
import frc.trigon.lib.commands.GearRatioCalculationCommand;
import frc.trigon.lib.commands.NetworkTablesCommand;
import frc.trigon.robot.RobotContainer;

import java.util.Set;
import java.util.function.Supplier;

public class TurretCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetFieldRelativeAngleDegrees) -> RobotContainer.TURRET.setTargetFieldRelativeAngle(Rotation2d.fromDegrees(targetFieldRelativeAngleDegrees)),
                false,
                Set.of(RobotContainer.TURRET),
                "Debugging/TurretTargetFieldRelativeAngleDegrees"
        );
    }

    public static Command getGearRatioCalibrationCommand() {
        return new GearRatioCalculationCommand(
                TurretConstants.MASTER_MOTOR,
                TurretConstants.ENCODER,
                0.5,
                RobotContainer.TURRET
        );
    }

    public static Command getAlignToHubCommand() {
        return new ExecuteEndCommand(
                RobotContainer.TURRET::alignToHub,
                RobotContainer.TURRET::stop,
                RobotContainer.TURRET
        );
    }

    public static Command getAlignToClosestAprilTagCommand() {
        return new ExecuteEndCommand(
                RobotContainer.TURRET::alignToClosestAprilTag,
                RobotContainer.TURRET::stop,
                RobotContainer.TURRET
        );
    }

    public static Command getAlignForDeliveryCommand() {
        return new ExecuteEndCommand(
                RobotContainer.TURRET::alignForDelivery,
                RobotContainer.TURRET::stop,
                RobotContainer.TURRET
        );
    }

    public static Command getAlignForEjectionCommand() {
        return new StartEndCommand(
                RobotContainer.TURRET::alignForEjection,
                RobotContainer.TURRET::stop,
                RobotContainer.TURRET
        );
    }

    public static Command getStopCommand() {
        return new StartEndCommand(
                RobotContainer.TURRET::stop,
                () -> {
                },
                RobotContainer.TURRET
        );
    }

    public static Command getSetTargetFieldRelativeAngleCommand(Supplier<Rotation2d> targetAngle) {
        return new ExecuteEndCommand(
                () -> RobotContainer.TURRET.setTargetFieldRelativeAngle(targetAngle.get()),
                RobotContainer.TURRET::stop,
                RobotContainer.TURRET
        );
    }

    public static Command getSetTargetSelfRelativeAngleAngleCommand(Supplier<Rotation2d> targetAngle) {
        return new ExecuteEndCommand(
                () -> RobotContainer.TURRET.setTargetSelfRelativeAngle(targetAngle.get()),
                RobotContainer.TURRET::stop,
                RobotContainer.TURRET
        );
    }
}