package frc.trigon.robot.commands.commandfactories.autonomous;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.gamepieceautodrive.GamePieceAutoDriveCommand;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.commands.commandfactories.ShootingCommands;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.misc.shootingphysics.ShootingCalculations;
import frc.trigon.robot.misc.shootingphysics.ShootingState;
import frc.trigon.robot.subsystems.hood.HoodCommands;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.turret.TurretCommands;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.function.Supplier;

/**
 * A class that contains command factories for preparation commands and commands used during the 20-second autonomous period at the start of each match.
 */
public class GeneralAutonomousCommands {
    public static Command getDeliveryCommand(AutonomousGenerator.AutonomousState previousState, double collectionTimeout) {
        return new ParallelDeadlineGroup(
                new ConditionalCommand(
                        new GamePieceAutoDriveCommand(true).withTimeout(collectionTimeout),
                        getDriveToFuelInNeutralZoneCommand(previousState == null, collectionTimeout),
                        () -> previousState != null && !previousState.isInAllianceZone
                ),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.INTAKE),
                GeneralCommands.getContinuousConditionalCommand(
                        ShootingCommands.getShootAtHubCommand(),
                        ShootingCommands.getDeliveryCommand(),
                        SafeAutonomousDriveCommands::isInAllianceZone
                )
        );
    }

    public static Command getCollectFromNeutralZoneCommand(AutonomousGenerator.AutonomousState previousState, double collectionTimeout) {
        return new ParallelDeadlineGroup(
                new ConditionalCommand(
                        new GamePieceAutoDriveCommand(true).withTimeout(collectionTimeout),
                        getDriveToFuelInNeutralZoneCommand(previousState == null, collectionTimeout),
                        () -> previousState != null && !previousState.isInAllianceZone
                ),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.INTAKE),
                getShootAtHubWhileDrivingCommand()
        );
    }

    public static Command getScoreCommand(AutonomousGenerator.AutonomousState nextState, double timeout) {
        return new ParallelCommandGroup(
                SafeAutonomousDriveCommands.getSafeDriveToPoseCommand(
                        () -> getScoringPose(nextState),
                        AutonomousConstants.DRIVE_IN_AUTONOMOUS_CONSTRAINTS,
                        0,
                        AutonomousConstants.DRIVE_SLOWLY_IN_AUTONOMOUS_CONSTRAINTS,
                        1
                ),
                getShootAtHubWhileDrivingCommand()
        ).withTimeout(timeout);
    }

    public static Command getCollectFromDepotCommand(boolean shootWhileDriving, double collectionTimeout) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        SafeAutonomousDriveCommands.getSafeDriveToPoseCommand(
                                () -> FieldConstants.DEPOT_POSITION,
                                AutonomousConstants.DRIVE_IN_AUTONOMOUS_CONSTRAINTS,
                                0,
                                AutonomousConstants.DRIVE_SLOWLY_IN_AUTONOMOUS_CONSTRAINTS,
                                shootWhileDriving ? 1 : 0
                        ),
                        getShootAtHubWhileDrivingCommand()
                ).until(() -> RobotContainer.SWERVE.atPose(FieldConstants.DEPOT_POSITION)),
                new GamePieceAutoDriveCommand(true).alongWith(ShootingCommands.getShootAtHubCommand()).withTimeout(collectionTimeout)
        ).alongWith(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.INTAKE));
    }

    public static Command getClimbCommand(Supplier<FlippablePose2d> climbPosition) {
        return new SequentialCommandGroup(
                SafeAutonomousDriveCommands.getSafeDriveToPoseCommand(
                        climbPosition,
                        AutonomousConstants.DRIVE_IN_AUTONOMOUS_CONSTRAINTS,
                        0,
                        AutonomousConstants.DRIVE_SLOWLY_IN_AUTONOMOUS_CONSTRAINTS,
                        1
                ).raceWith(getShootAtHubWhileDrivingCommand()),
                new InstantCommand() // TODO: Add climb command
        );
    }

    private static Command getShootAtHubWhileDrivingCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                ShootingCommands.getShootAtHubCommand(),
                getPrepareForShootingCommand(),
                SafeAutonomousDriveCommands::isInAllianceZone
        );
    }

    private static Command getDriveToFuelInNeutralZoneCommand(boolean shootPreload, double timeout) {
        return new SequentialCommandGroup(
                SafeAutonomousDriveCommands.getSafeDriveToPoseCommand(
                        () -> SafeAutonomousDriveCommands.isRight() ? FieldConstants.RIGHT_INTAKE_POSITION : FieldConstants.LEFT_INTAKE_POSITION,
                        AutonomousConstants.DRIVE_IN_AUTONOMOUS_CONSTRAINTS,
                        3,
                        AutonomousConstants.SHOOT_PRELOAD_BEFORE_NEUTRAL_ZONE_DRIVE_CONSTRAINTS,
                        shootPreload ? AutonomousConstants.SHOOT_PRELOAD_BEFORE_NEUTRAL_ZONE_DRIVE_TIME : 0
                ).until(GeneralAutonomousCommands::shouldRobotStartIntaking),
                new GamePieceAutoDriveCommand(false).withTimeout(timeout)
        );
    }

    private static Command getPrepareForShootingCommand() {
        return getAimWithTargetShootingState(
                () -> ShootingCalculations.getInstance().calculateTargetShootingState(
                        SafeAutonomousDriveCommands.isRight() ? FieldConstants.RIGHT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE.get() : FieldConstants.LEFT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE.get(),
                        new Translation2d()
                )
        );
    }

    private static Command getAimWithTargetShootingState(Supplier<ShootingState> targetShootingState) {
        return new ParallelCommandGroup(
                TurretCommands.getSetTargetFieldRelativeAngleCommand(() -> targetShootingState.get().targetFieldRelativeYaw()),
                HoodCommands.getSetTargetAngleCommand(() -> targetShootingState.get().targetPitch())
        );
    }

    private static boolean shouldRobotStartIntaking() {
        final Pose2d currentRobotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        return Flippable.isRedAlliance() ? currentRobotPose.getX() < (FieldConstants.FIELD_LENGTH_METERS - AutonomousConstants.START_INTAKING_X) : currentRobotPose.getX() > AutonomousConstants.START_INTAKING_X;
    }

    private static FlippablePose2d getScoringPose(AutonomousGenerator.AutonomousState nextState) {
        if (nextState == null && AutonomousGenerator.shouldClimb())
            return AutonomousGenerator.CLIMB_POSITION_CHOOSER.get().climbPose;
        if (nextState != null && !nextState.isInAllianceZone)
            return SafeAutonomousDriveCommands.isRight() ? FieldConstants.RIGHT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE : FieldConstants.LEFT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE;
        if (nextState == AutonomousGenerator.AutonomousState.COLLECT_FROM_DEPOT)
            return FieldConstants.DEPOT_POSITION;
        return SafeAutonomousDriveCommands.isRight() ? FieldConstants.RIGHT_IDEAL_SHOOTING_POSITION : FieldConstants.LEFT_IDEAL_SHOOTING_POSITION;
    }

    /**
     * Creates a command that resets the pose estimator's pose to the starting pose of the given autonomous as long as the robot is not enabled.
     *
     * @param autoName the name of the autonomous
     * @return a command that resets the robot's pose estimator pose to the start position of the given autonomous
     */
    public static Command getResetPoseToAutoPoseCommand(Supplier<String> autoName) {
        return new InstantCommand(
                () -> {
                    if (DriverStation.isEnabled())
                        return;
                    RobotContainer.ROBOT_POSE_ESTIMATOR.resetPose(getAutoStartPose(autoName.get()));
                }
        ).ignoringDisable(true);
    }

    /**
     * Gets the starting position of the target PathPlanner autonomous.
     *
     * @param autoName the name of the autonomous group
     * @return the staring pose of the autonomous
     */
    public static Pose2d getAutoStartPose(String autoName) {
        try {
            final Pose2d nonFlippedAutoStartPose = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingHolonomicPose().get();
            final FlippablePose2d flippedAutoStartPose = new FlippablePose2d(nonFlippedAutoStartPose, true);
            return flippedAutoStartPose.get();
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            return new Pose2d();
        }
    }
}