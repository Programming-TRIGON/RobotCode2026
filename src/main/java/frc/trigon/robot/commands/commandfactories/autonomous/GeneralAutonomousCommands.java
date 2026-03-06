package frc.trigon.robot.commands.commandfactories.autonomous;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.gamepieceautodrive.GamePieceAutoDriveCommand;
import frc.trigon.robot.commands.commandfactories.ClimbCommands;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.commands.commandfactories.ShootingCommands;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.shootingphysics.ShootingCalculations;
import frc.trigon.robot.misc.shootingphysics.ShootingState;
import frc.trigon.robot.subsystems.climber.ClimberCommands;
import frc.trigon.robot.subsystems.climber.ClimberConstants;
import frc.trigon.robot.subsystems.hood.HoodCommands;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.subsystems.turret.TurretCommands;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.function.Supplier;

/**
 * A class that contains command factories for preparation commands and commands used during the 20-second autonomous period at the start of each match.
 */
public class GeneralAutonomousCommands {
    public static Command getDeliveryCommand(AutonomousGenerator.AutonomousState previousState, Supplier<Double> collectionTimeout) {
        return new ParallelDeadlineGroup(
                new ConditionalCommand(
                        getDriveToFuelCommand(true).withTimeout(collectionTimeout.get()),
                        getDriveToFuelInNeutralZoneCommand(
                                previousState == null,
                                collectionTimeout.get(),
                                true,
                                SafeAutonomousDriveCommands.isRight() ? FieldConstants.RIGHT_START_INTAKING_FOR_DELIVERY_POSITION : FieldConstants.LEFT_START_INTAKING_FOR_DELIVERY_POSITION,
                                true
                        ),
                        () -> previousState != null && !previousState.isInAllianceZone
                ),
                new WaitUntilCommand(GeneralAutonomousCommands::isInTrenchYRange).onlyIf(FieldConstants::isInAllianceZone).andThen(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.INTAKE)),
                GeneralCommands.getContinuousConditionalCommand(
                        ShootingCommands.getShootAtHubCommand(),
                        ShootingCommands.getDeliveryCommand(),
                        FieldConstants::isInAllianceZone
                )
        );
    }

    public static Command getCollectFromNeutralZoneCommand(AutonomousGenerator.AutonomousState previousState, double collectionTimeout) {
        return new ParallelDeadlineGroup(
                new ConditionalCommand(
                        getDriveToFuelCommand(false).withTimeout(collectionTimeout),
                        getDriveToFuelInNeutralZoneCommand(
                                previousState == null,
                                collectionTimeout,
                                false,
                                SafeAutonomousDriveCommands.isRight() ? FieldConstants.RIGHT_INTAKE_POSITION : FieldConstants.LEFT_INTAKE_POSITION,
                                false
                        ),
                        () -> previousState != null && !previousState.isInAllianceZone
                ),
                new WaitUntilCommand(GeneralAutonomousCommands::isInTrenchYRange).onlyIf(FieldConstants::isInAllianceZone).andThen(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.INTAKE)),
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
                        1000
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
                                shootWhileDriving ? 1000 : 0
                        ),
                        getShootAtHubWhileDrivingCommand()
                ).until(() -> RobotContainer.SWERVE.atPose(FieldConstants.DEPOT_POSITION)),
                new WaitCommand(0.1),
                getDriveToFuelCommand(true).alongWith(ShootingCommands.getShootAtHubCommand()).withTimeout(collectionTimeout)
        ).alongWith(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.INTAKE));
    }

    public static Command getClimbCommand(Supplier<FlippablePose2d> climbPosition) {
        return new ParallelCommandGroup(
                SafeAutonomousDriveCommands.getSafeDriveToPoseCommand(
                                climbPosition,
                                AutonomousConstants.DRIVE_IN_AUTONOMOUS_CONSTRAINTS,
                                0,
                                AutonomousConstants.DRIVE_SLOWLY_IN_AUTONOMOUS_CONSTRAINTS,
                                1000
                        ).raceWith(getShootAtHubWhileDrivingCommand())
                        .until(() -> RobotContainer.SWERVE.atPose(climbPosition.get()))
                        .andThen(SwerveCommands.getClosedLoopFieldRelativeDriveCommand(() -> 0, () -> 0, () -> 0)),
                getClimbToL1Command(climbPosition)
        ).onlyIf(() -> climbPosition.get() != null);
    }

    private static Command getClimbToL1Command(Supplier<FlippablePose2d> climbPosition) {
        return new SequentialCommandGroup(
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.PREPARE_CLIMB).until(() -> RobotContainer.CLIMBER.atTargetState() && RobotContainer.SWERVE.atPose(climbPosition.get())),
                new InstantCommand(() -> ClimbCommands.IS_CLIMBING.set(true)),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB_L1)
        ).until(OperatorConstants.CANCEL_CLIMB_TRIGGER);
    }

    private static Command getShootAtHubWhileDrivingCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                getPrepareForShootingWithoutHoodCommand(),
                GeneralCommands.getContinuousConditionalCommand(
                        ShootingCommands.getShootAtHubCommand(),
                        getPrepareForShootingCommand(),
                        FieldConstants::isInAllianceZone
                ),
                GeneralAutonomousCommands::isInTrenchXRange
        );
    }

    static boolean isInTrenchXRange() {
        final Pose2d currentRobotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        double entryXFromAllianceZone = isAngleCloserTo180(currentRobotPose.getRotation()) ^ Flippable.isRedAlliance() ? 4.25 : 4.1;
        double entryXFromNeutralZone = isAngleCloserTo180(currentRobotPose.getRotation()) ^ Flippable.isRedAlliance() ? 5.4 : 5.6;
        entryXFromAllianceZone = Flippable.isRedAlliance() ? FieldConstants.FIELD_LENGTH_METERS - entryXFromAllianceZone : entryXFromAllianceZone;
        entryXFromNeutralZone = Flippable.isRedAlliance() ? FieldConstants.FIELD_LENGTH_METERS - entryXFromNeutralZone : entryXFromNeutralZone;
        return currentRobotPose.getX() > entryXFromAllianceZone && currentRobotPose.getX() < entryXFromNeutralZone
                || currentRobotPose.getX() < entryXFromAllianceZone && currentRobotPose.getX() > entryXFromNeutralZone;
    }

    static boolean isInTrenchYRange() {
        final Pose2d currentRobotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        return currentRobotPose.getY() > FieldConstants.LEFT_TRENCH_MIN_Y || currentRobotPose.getY() < FieldConstants.RIGHT_TRENCH_MAX_Y;
    }

    private static boolean isAngleCloserTo180(Rotation2d angle) {
        return Math.abs(angle.getDegrees()) > 90;
    }

    private static Command getDriveToFuelInNeutralZoneCommand(boolean shootPreload, double timeout, boolean shouldWaitUntilAtPose, FlippablePose2d targetPose, boolean intakeSlowly) {
        return new SequentialCommandGroup(
                SafeAutonomousDriveCommands.getSafeDriveToPoseCommand(
                        () -> targetPose,
                        AutonomousConstants.DRIVE_FOR_INTAKING_CONSTRAINTS,
                        3,
                        AutonomousConstants.SHOOT_PRELOAD_BEFORE_NEUTRAL_ZONE_DRIVE_CONSTRAINTS,
                        shootPreload ? AutonomousConstants.SHOOT_PRELOAD_BEFORE_NEUTRAL_ZONE_TIME_SECONDS : 0
                ).until(shouldWaitUntilAtPose ? () -> RobotContainer.SWERVE.atPose(targetPose) : GeneralAutonomousCommands::shouldRobotStartIntaking),
                getDriveToFuelCommand(intakeSlowly).withTimeout(timeout)
        );
    }

    private static Command getDriveToFuelCommand(boolean intakeSlowly) {
        return GeneralCommands.getContinuousConditionalCommand(
                new GamePieceAutoDriveCommand(intakeSlowly),
                SwerveCommands.getClosedLoopSelfRelativeDriveCommand(() -> 0, () -> 0, Flippable.isRedAlliance() ? () -> 0.2 : () -> -0.2),
                GamePieceAutoDriveCommand::hasCollectableGamePiecesInView
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

    private static Command getPrepareForShootingWithoutHoodCommand() {
        return HoodCommands.getRestCommand();
    }

    private static Command getAimWithTargetShootingState(Supplier<ShootingState> targetShootingState) {
        return new ParallelCommandGroup(
                TurretCommands.getSetTargetFieldRelativeAngleCommand(() -> targetShootingState.get().targetFieldRelativeYaw()),
                HoodCommands.getSetTargetAngleCommand(() -> targetShootingState.get().targetPitch())
        );
    }

    private static boolean shouldRobotStartIntaking() {
        final Pose2d currentRobotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        if (Flippable.isRedAlliance())
            return currentRobotPose.getX() < (FieldConstants.FIELD_LENGTH_METERS - AutonomousConstants.START_INTAKING_X);
        return currentRobotPose.getX() > AutonomousConstants.START_INTAKING_X;
    }

    static FlippablePose2d getScoringPose(AutonomousGenerator.AutonomousState nextState) {
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