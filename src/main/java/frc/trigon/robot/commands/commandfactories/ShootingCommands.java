package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.commands.WaitUntilChangeCommand;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.lib.utilities.flippable.FlippableRotation2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.misc.shootingphysics.ShootingCalculations;
import frc.trigon.robot.misc.shootingphysics.ShootingState;
import frc.trigon.robot.subsystems.hood.HoodCommands;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.loader.LoaderCommands;
import frc.trigon.robot.subsystems.loader.LoaderConstants;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.spindexer.SpindexerCommands;
import frc.trigon.robot.subsystems.spindexer.SpindexerConstants;
import frc.trigon.robot.subsystems.turret.TurretCommands;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ShootingCommands {
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();
    private static ShootingState FIXED_HUB_SHOOTING_STATE = FixedShootingPosition.CLOSE_TO_HUB.targetState;

    public static Command getShortEjectFuelCommand() {
        return new ParallelCommandGroup(
                SpindexerCommands.getSetTargetStateCommand(SpindexerConstants.SpindexerState.LOAD_FOR_EJECT),
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.LOAD_FOR_EJECT),
                TurretCommands.getAlignForEjectionCommand(),
                HoodCommands.getAimForEjectionCommand(),
                ShooterCommands.getAimForEjectionCommand()
        );
    }

    public static Command getUnjamCommand() {
        return new ParallelCommandGroup(
                SpindexerCommands.getSetTargetStateCommand(SpindexerConstants.SpindexerState.UNJAM),
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.UNJAM),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.EJECT)
        );
    }

    public static Command getShootAtHubCommand() {
        return new ParallelCommandGroup(
                getAimAtHubCommand(),
                getLoadFuelWhenReadyCommand(true, false)
        );
    }

    public static Command getDeliveryCommand() {
        return new ParallelCommandGroup(
                getAimForDeliveryCommand(),
                getLoadFuelWhenReadyCommand(false, true)
        );
    }

    public static Command getFixedHubShootingCommand() {
        return getShootFromFixedPositionCommand(() -> FIXED_HUB_SHOOTING_STATE, false);
    }

    public static Command getFixedDeliveryCommand() {
        return getShootFromFixedPositionCommand(() -> FixedShootingPosition.FIXED_DELIVERY.targetState, true);
    }

    public static Command getChangeFixedShootingPositionCommand(FixedShootingPosition fixedPosition) {
        return new InstantCommand(() -> updateFixedShootingPosition(fixedPosition)).ignoringDisable(true);
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

    public static Command getAimAtHubWithoutHoodCommand() {
        return new InstantCommand(ShootingCommands::updateShootingCalculations).andThen(
                new ParallelCommandGroup(
                        new RunCommand(ShootingCommands::updateShootingCalculations),
                        TurretCommands.getAlignToHubCommand(),
                        HoodCommands.getRestCommand(),
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

    private static Command getShootFromFixedPositionCommand(Supplier<ShootingState> fixedShootingStateSupplier, boolean isDelivery) {
        return new ParallelCommandGroup(
                getAimForFixedStateCommand(fixedShootingStateSupplier)
                        .raceWith(new WaitUntilChangeCommand<>(fixedShootingStateSupplier)).repeatedly(),
                getLoadFuelWhenReadyCommand(false, isDelivery)
        );
    }

    private static Command getAimForFixedStateCommand(Supplier<ShootingState> fixedShootingStateSupplier) {
        return new ParallelCommandGroup(
                TurretCommands.getSetTargetFieldRelativeAngleCommand(() -> new FlippableRotation2d(fixedShootingStateSupplier.get().targetFieldRelativeYaw(), true).get()),
                HoodCommands.getSetTargetAngleCommand(() -> fixedShootingStateSupplier.get().targetPitch()),
                ShooterCommands.getSetTargetVelocityCommand(() -> fixedShootingStateSupplier.get().targetShootingVelocityMetersPerSecond())
        );
    }

    private static Command getLoadFuelWhenReadyCommand(boolean isAutoShootingAtHub, boolean isDelivery) {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> canShoot(isAutoShootingAtHub) && (!isDelivery || !isDeliveryHittingHub()) && !isShotHittingTower()),
                getLoadFuelCommand(isAutoShootingAtHub).until(() -> ShootingCommands.shouldStopShooting(isAutoShootingAtHub, isDelivery))
        ).repeatedly().alongWith(new RunCommand(() -> logShouldStopShooting(isAutoShootingAtHub, isDelivery)));
    }

    private static Command getLoadFuelCommand(boolean isAutoShootingAtHub) {
        if (isAutoShootingAtHub) {
            return new ParallelCommandGroup(
                    SpindexerCommands.getLoadToShooterCommand(),
                    LoaderCommands.getLoadToShooterCommand()
            );
        }

        return new ParallelCommandGroup(
                SpindexerCommands.getSetTargetStateCommand(SpindexerConstants.SpindexerState.LOAD_FOR_DELIVERY),
                LoaderCommands.getSetTargetStateCommand(LoaderConstants.LoaderState.LOAD_FOR_DELIVERY)
        );
    }

    private static void updateFixedShootingPosition(FixedShootingPosition targetFixedShootingPosition) {
        FIXED_HUB_SHOOTING_STATE = targetFixedShootingPosition.targetState;
        Logger.recordOutput("FixedShootingPosition", targetFixedShootingPosition.name());
    }

    private static void logShouldStopShooting(boolean isShootingAtHub, boolean isDelivery) {
        Logger.recordOutput("Shooting/ShouldStopShooting", shouldStopShooting(isShootingAtHub, isDelivery));
        Logger.recordOutput("Shooting/IsDeliveryHittingHub", isDeliveryHittingHub());
        Logger.recordOutput("Shooting/IsShotHittingTower", isShotHittingTower());
    }

    private static boolean canShoot(boolean isShootingAtHub) {
        if (isShootingAtHub) {
            final boolean turretAtShootingCalculationsAngle = RobotContainer.TURRET.atTargetShootingCalculationsAngle(false);
            Logger.recordOutput("Shooting/Conditions/TurretAtSmallToleranceAngle", turretAtShootingCalculationsAngle);

            return canShootAtHub() && RobotContainer.SHOOTER.atTargetVelocity() && RobotContainer.HOOD.atTargetAngle() &&
                    turretAtShootingCalculationsAngle;
        }
        return RobotContainer.SHOOTER.atTargetVelocity()
                && RobotContainer.HOOD.atTargetAngle()
                && RobotContainer.TURRET.atTargetAngle(false);
    }

    private static boolean isShotHittingTower() {
        final Pose2d turretPose = RobotContainer.TURRET.getCurrentTurretFieldRelativePosition();

        final double minimumY = !Flippable.isRedAlliance() ? FieldConstants.TOWER_MINIMUM_Y : FieldConstants.FIELD_WIDTH_METERS - FieldConstants.TOWER_MAXIMUM_Y;
        final double maximumY = !Flippable.isRedAlliance() ? FieldConstants.TOWER_MAXIMUM_Y : FieldConstants.FIELD_WIDTH_METERS - FieldConstants.TOWER_MINIMUM_Y;
        Logger.recordOutput("Shooting/IsShotHittingTower/InYRange", turretPose.getY() < maximumY && turretPose.getY() > minimumY);
        Logger.recordOutput("Shooting/IsShotHittingTower/InXRange", turretPose.getX() < FieldConstants.TOWER_MAXIMUM_X || turretPose.getX() > FieldConstants.FIELD_LENGTH_METERS - FieldConstants.TOWER_MAXIMUM_X);
        return (turretPose.getY() < maximumY) &&
                (turretPose.getY() > minimumY) &&
                (turretPose.getX() < FieldConstants.TOWER_MAXIMUM_X || turretPose.getX() > FieldConstants.FIELD_LENGTH_METERS - FieldConstants.TOWER_MAXIMUM_X);
    }

    private static boolean isDeliveryHittingHub() {
        final Pose2d turretPose = RobotContainer.TURRET.getCurrentTurretFieldRelativePosition();
        final Rotation2d fieldRelativeTurretAngle = RobotContainer.TURRET.calculateTargetAngleForDelivery().plus(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getRotation());
        final double slope = Math.tan(fieldRelativeTurretAngle.getRadians());
        final double b = turretPose.getTranslation().getY() - (slope * turretPose.getTranslation().getX());
        final double netYOutsideHub = 0.25;

        final Translation2d hubPosition = FieldConstants.HUB_POSITION.get();
        final double halfSize = 1.19 / 2.0;
        final double minimumX = hubPosition.getX() - halfSize;
        final double maximumX = hubPosition.getX() + halfSize;
        final double minimumY = hubPosition.getY() - halfSize - netYOutsideHub;
        final double maximumY = hubPosition.getY() + halfSize + netYOutsideHub;

        double targetX = Flippable.isRedAlliance() ? minimumX : maximumX;

        double yAtTargetX = (slope * targetX) + b;

        Logger.recordOutput("Shooting/Delivery/DeliveryHittingHub/YAtTargetX", yAtTargetX);
        Logger.recordOutput("Shooting/Delivery/DeliveryHittingHub/MinimumY", minimumY);
        Logger.recordOutput("Shooting/Delivery/DeliveryHittingHub/MaximumY", maximumY);
        Logger.recordOutput("Shooting/Delivery/DeliveryHittingHub/TargetX", targetX);
        Logger.recordOutput("Shooting/Delivery/DeliveryHittingHub/slope", slope);
        Logger.recordOutput("Shooting/Delivery/DeliveryHittingHub/b", b);

        return (yAtTargetX <= maximumY) && (yAtTargetX >= minimumY);
    }

    private static boolean canShootAtHub() {
        return RobotContainer.SHOOTER.isAimingAtHub();
    }

    private static boolean shouldStopShooting(boolean isShootingAtHub, boolean isDelivery) {
        return (isShootingAtHub ? !RobotContainer.TURRET.atTargetShootingCalculationsAngle(true) || isShotHittingTower() : !RobotContainer.TURRET.atTargetAngle(true)) && (!isDelivery || isDeliveryHittingHub());
    }

    private static void updateShootingCalculations() {
        SHOOTING_CALCULATIONS.updateCalculations();
    }

    public enum FixedShootingPosition {//TODO: Get all values from shooting calculations
        CLOSE_TO_HUB(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0),
        LEFT_CORNER(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0),
        CLOSE_TO_TOWER(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0),
        CLOSE_TO_OUTPOST(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0),
        FIXED_DELIVERY(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(50), 6);

        private final ShootingState targetState;

        FixedShootingPosition(Rotation2d targetFieldRelativeYaw, Rotation2d targetPitch,
                              double targetShootingVelocityMetersPerSecond) {
            this.targetState = new ShootingState(targetFieldRelativeYaw, targetPitch, targetShootingVelocityMetersPerSecond);
        }
    }
}