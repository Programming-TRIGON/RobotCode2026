package frc.trigon.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
                (targetAngleDegrees) -> RobotContainer.TURRET.setTargetAngle(Rotation2d.fromDegrees(targetAngleDegrees)),
                false,
                Set.of(RobotContainer.TURRET),
                "Debugging/TurretTargetAngleDegrees"
        );
    }

    public static Command getGearRatioCalibrationCommand() {
        return new GearRatioCalculationCommand(
                TurretConstants.MASTER_MOTOR,
                TurretConstants.ENCODER,
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

    public static Command getStopCommand() {
        return new StartEndCommand(
                RobotContainer.TURRET::stop,
                () -> {
                },
                RobotContainer.TURRET
        );
    }

    public static Command getSetTargetAngleCommand(Supplier<Rotation2d> targetAngle) {
        return new RunCommand(
                () -> RobotContainer.TURRET.setTargetAngle(targetAngle.get()),
                RobotContainer.TURRET
        );
    }
}