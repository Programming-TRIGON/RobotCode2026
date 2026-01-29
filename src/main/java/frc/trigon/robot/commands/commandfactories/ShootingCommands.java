package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.commands.WaitUntilChangeCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.shootingphysics.ShootingCalculations;
import frc.trigon.robot.misc.shootingphysics.ShootingState;
import frc.trigon.robot.subsystems.hood.HoodCommands;
import frc.trigon.robot.subsystems.loader.LoaderCommands;
import frc.trigon.robot.subsystems.loader.LoaderConstants;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.spindexer.SpindexerCommands;
import frc.trigon.robot.subsystems.spindexer.SpindexerConstants;
import frc.trigon.robot.subsystems.turret.TurretCommands;

public class ShootingCommands {
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();
    private static ShootingState FIXED_SHOOTING_STATE = FixedShootingPosition.CLOSE_TO_HUB.targetState;

    public static Command getShortEjectFuelCommand() {
        return new ParallelCommandGroup(
                SpindexerCommands.getSetTargetStateCommand(SpindexerConstants.SpindexerState.LOAD_TO_TURRET),
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.LOAD_FOR_EJECT),
                TurretCommands.getAlignForEjectionCommand(),
                HoodCommands.getAimForEjectionCommand(),
                ShooterCommands.getAimForEjectionCommand()
        );
    }

    public static Command getShootAtHubCommand() {
        return new ParallelCommandGroup(
                getAimAtHubCommand(),
                getLoadFuelWhenReadyCommand(true)
        );
    }

    public static Command getShootFromFixedPositionCommand() {
        return new ParallelCommandGroup(
                getAimForFixedStateCommand()
                        .raceWith(new WaitUntilChangeCommand<>(() -> FIXED_SHOOTING_STATE)).repeatedly(),
                getLoadFuelWhenReadyCommand(false)
        );
    }

    public static Command getDeliveryCommand(boolean isFixedDelivery) {
        return new ParallelCommandGroup(
                isFixedDelivery ? getAimForFixedDeliveryCommand() : getAimForDeliveryCommand(),
                getLoadFuelWhenReadyCommand(false)
        );
    }

    public static Command getChangeFixedShootingPositionCommand(FixedShootingPosition fixedPosition) {
        return new InstantCommand(() -> FIXED_SHOOTING_STATE = fixedPosition.targetState);
    }

    private static Command getAimAtHubCommand() {
        return new InstantCommand(ShootingCommands::updateShootingCalculations).andThen(
                new ParallelCommandGroup(
                        new RunCommand(ShootingCommands::updateShootingCalculations),
                        TurretCommands.getAlignToHubCommand(),
                        HoodCommands.getAimAtHubCommand(),
                        ShooterCommands.getAimAtHubCommand()
                )
        );
    }

    private static Command getAimForDeliveryCommand() {
        return new ParallelCommandGroup(
                TurretCommands.getAlignForDeliveryCommand(),
                HoodCommands.getAimForDeliveryCommand(),
                ShooterCommands.getAimForDeliveryCommand()
        );
    }

    private static Command getAimForFixedStateCommand() {
        return new ParallelCommandGroup(
                TurretCommands.getSetTargetFieldRelativeAngleCommand(FIXED_SHOOTING_STATE::targetFieldRelativeYaw),
                HoodCommands.getSetTargetAngleCommand(FIXED_SHOOTING_STATE.targetPitch()),
                ShooterCommands.getSetTargetVelocityCommand(FIXED_SHOOTING_STATE.targetShootingVelocityMetersPerSecond())
        );
    }

    private static Command getAimForFixedDeliveryCommand() {
        return new ParallelCommandGroup(
                TurretCommands.getAlignForEjectionCommand(),
                HoodCommands.getAimForEjectionCommand(),
                ShooterCommands.getAimForFixedDeliveryCommand()
        );
    }

    private static Command getLoadFuelWhenReadyCommand(boolean isShootingAtHub) {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> canShoot(isShootingAtHub)),
                getLoadFuelCommand()
                        .until(ShootingCommands::shouldStopShooting)
        ).repeatedly();
    }

    private static Command getLoadFuelCommand() {
        return new ParallelCommandGroup(
                SpindexerCommands.getSetTargetStateCommand(SpindexerConstants.SpindexerState.LOAD_TO_TURRET),
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.LOAD)
        );
    }

    private static boolean canShoot(boolean isShootingAtHub) {
        return (!isShootingAtHub || canShootAtHub()) &&
                RobotContainer.SHOOTER.atTargetVelocity()
                && RobotContainer.HOOD.atTargetAngle()
                && RobotContainer.TURRET.atTargetAngle(false);
    }

    private static boolean canShootAtHub() {
        return RobotContainer.SHOOTER.isAimingAtHub();
    }

    private static boolean shouldStopShooting() {
        return !RobotContainer.TURRET.atTargetAngle(true);
    }

    private static void updateShootingCalculations() {
        SHOOTING_CALCULATIONS.updateCalculations();
    }

    public enum FixedShootingPosition {//TODO: Get all values from shooting calculations
        CLOSE_TO_HUB(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0),
        LEFT_CORNER(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0),
        CLOSE_TO_TOWER(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0),
        CLOSE_TO_OUTPOST(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0);

        private final ShootingState targetState;

        FixedShootingPosition(Rotation2d targetFieldRelativeYaw, Rotation2d targetPitch,
                              double targetShootingVelocityMetersPerSecond) {
            this.targetState = new ShootingState(targetFieldRelativeYaw, targetPitch, targetShootingVelocityMetersPerSecond);
        }
    }
}