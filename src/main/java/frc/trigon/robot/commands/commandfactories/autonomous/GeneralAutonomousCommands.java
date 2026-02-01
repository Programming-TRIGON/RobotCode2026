package frc.trigon.robot.commands.commandfactories.autonomous;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.gamepieceautodrive.GamePieceAutoDriveCommand;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.commands.commandfactories.ShootingCommands;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.function.Supplier;

/**
 * A class that contains command factories for preparation commands and commands used during the 20-second autonomous period at the start of each match.
 */
public class GeneralAutonomousCommands {
    public static Command getDeliveryCommand(AutonomousGenerator.AutonomousState previousState, double collectionTimeout) {
        return new ParallelDeadlineGroup(
                getDriveToFuelInNeutralZoneCommand(previousState == null, collectionTimeout),
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
                getDriveToFuelInNeutralZoneCommand(previousState == null, collectionTimeout),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.INTAKE),
                ShootingCommands.getShootAtHubCommand().onlyWhile(SafeAutonomousDriveCommands::isInAllianceZone)
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
                ShootingCommands.getShootAtHubCommand().onlyWhile(SafeAutonomousDriveCommands::isInAllianceZone).repeatedly()
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
                        ShootingCommands.getShootAtHubCommand().onlyWhile(SafeAutonomousDriveCommands::isInAllianceZone).repeatedly()
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
                        1000
                ),
                new InstantCommand() // TODO: Add climb command
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
                ),
                new GamePieceAutoDriveCommand(false).withTimeout(timeout)
        );
    }

    private static FlippablePose2d getScoringPose(AutonomousGenerator.AutonomousState nextState) {
        if (nextState == null && AutonomousGenerator.shouldClimb())
            return AutonomousGenerator.CLIMB_POSITION_CHOOSER.get().climbPose;
        return SafeAutonomousDriveCommands.isRight() ? FieldConstants.RIGHT_IDEAL_SHOOTING_POSITION : FieldConstants.LEFT_IDEAL_SHOOTING_POSITION;
    }

    private static boolean shouldCollectGamePiecesFromNeutralZone() {
        final Pose2d currentRobotPose = new FlippablePose2d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose(), true).get();
        return currentRobotPose.getX() > FieldConstants.DELIVERY_ZONE_START_BLUE_X && RobotContainer.OBJECT_POSE_ESTIMATOR.hasObjects();
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