package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.commands.WaitUntilChangeCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.OperatorConstants;
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
    public static boolean SHOULD_SHOOT_FROM_SET_POSITION = false;
    public static ShootingState SET_SHOOTING_STATE = SetShootingPosition.CLOSE_TO_HUB.targetState;

    public static Command getShootAtHubCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(ShootingCommands::updateShootingCalculations),
                new ParallelCommandGroup(
                        getAimAtHubCommand(),
                        getLoadFuelWhenReadyCommand()
                )
        );
    }

    public static Command getShootFromSetPositionCommand() {
        return new ParallelCommandGroup(
                getAimForSetStateCommand()
                        .raceWith(new WaitUntilChangeCommand<>(() -> SET_SHOOTING_STATE)).repeatedly(),
                getLoadFuelWhenReadyCommand()
        );
    }

    public static Command getDeliveryCommand() {
        return new ParallelCommandGroup(
                getAimForDeliveryCommand(),
                getLoadFuelWhenReadyCommand()
        );
    }

    public static Command getToggleShouldShootFromSetPositionCommand() {
        return new InstantCommand(() -> ShootingCommands.SHOULD_SHOOT_FROM_SET_POSITION = !ShootingCommands.SHOULD_SHOOT_FROM_SET_POSITION);
    }

    public static Command getChangeSetShootingPositionCommand(SetShootingPosition setPosition) {
        return new InstantCommand(() -> SET_SHOOTING_STATE = setPosition.targetState);
    }

    private static Command getAimAtHubCommand() {
        return new ParallelCommandGroup(
                new RunCommand(ShootingCommands::updateShootingCalculations),
                TurretCommands.getAlignToHubCommand(),
                HoodCommands.getAimAtHubCommand(),
                ShooterCommands.getAimAtHubCommand()
        );
    }

    private static Command getAimForDeliveryCommand() {
        return new ParallelCommandGroup(
                TurretCommands.getAlignForDeliveryCommand(),
                HoodCommands.getAimForDeliveryCommand(),
                ShooterCommands.getAimForDeliveryCommand()
        );
    }

    private static Command getAimForSetStateCommand() {
        return new ParallelCommandGroup(
                TurretCommands.getSetTargetFieldRelativeAngleCommand(SET_SHOOTING_STATE::targetFieldRelativeYaw),
                HoodCommands.getSetTargetAngleCommand(SET_SHOOTING_STATE.targetPitch()),
                ShooterCommands.getSetTargetVelocityCommand(SET_SHOOTING_STATE.targetShootingVelocityMetersPerSecond())
        );
    }

    private static Command getLoadFuelWhenReadyCommand() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(ShootingCommands::canShoot),
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

    private static boolean canShoot() {
        return isRobotReadyToShoot() || OperatorConstants.OVERRIDE_CAN_SHOOT_TRIGGER.getAsBoolean();
    }

    private static boolean shouldStopShooting() {
        return RobotContainer.TURRET.atTargetAngle(true) && !OperatorConstants.OVERRIDE_CAN_SHOOT_TRIGGER.getAsBoolean();
    }

    private static boolean isRobotReadyToShoot() {
        return RobotContainer.SHOOTER.atTargetVelocity()
                && RobotContainer.HOOD.atTargetAngle()
                && RobotContainer.TURRET.atTargetAngle(false);
    }

    private static void updateShootingCalculations() {
        ShootingCalculations.getInstance().updateCalculations();
    }

    public enum SetShootingPosition {
        CLOSE_TO_HUB(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0),
        LEFT_CORNER(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0),
        CLOSE_TO_TOWER(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0),
        CLOSE_TO_OUTPOST(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0);

        private final ShootingState targetState;

        SetShootingPosition(Rotation2d targetFieldRelativeYaw, Rotation2d targetPitch,
                            double targetShootingVelocityMetersPerSecond) {
            this.targetState = new ShootingState(targetFieldRelativeYaw, targetPitch, targetShootingVelocityMetersPerSecond);
        }
    }
}