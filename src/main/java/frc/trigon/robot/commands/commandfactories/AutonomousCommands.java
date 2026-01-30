package frc.trigon.robot.commands.commandfactories;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.gamepieceautodrive.GamePieceAutoDriveCommand;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.function.Supplier;

/**
 * A class that contains command factories for preparation commands and commands used during the 15-second autonomous period at the start of each match.
 */
public class AutonomousCommands {
    public static Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                AutonomousConstants.FIRST_AUTONOMOUS_CHOOSER.get() == null ? new InstantCommand() : AutonomousConstants.FIRST_AUTONOMOUS_CHOOSER.get().get(),
                AutonomousConstants.SECOND_AUTONOMOUS_CHOOSER.get() == null ? new InstantCommand() : AutonomousConstants.SECOND_AUTONOMOUS_CHOOSER.get().get(),
                AutonomousConstants.THIRD_AUTONOMOUS_CHOOSER.get() == null ? new InstantCommand() : AutonomousConstants.THIRD_AUTONOMOUS_CHOOSER.get().get(),
                getClimbCommand(AutonomousConstants.CLIMB_POSITION_CHOOSER.get()).onlyIf(() -> AutonomousConstants.CLIMB_POSITION_CHOOSER.get() != null)
        );
    }

    public static Command getDeliveryCommand() {
        return new ParallelCommandGroup(
                getSafeDriveToPoseCommand(isRight() ? () -> FieldConstants.RIGHT_INTAKE_POSITION : () -> FieldConstants.LEFT_INTAKE_POSITION),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.INTAKE),
                GeneralCommands.getContinuousConditionalCommand(
                        ShootingCommands.getShootAtHubCommand(),
                        ShootingCommands.getDeliveryCommand(),
                        AutonomousCommands::isInAllianceZone
                )
        ).withTimeout(AutonomousConstants.DELIVERY_TIMEOUT_SECONDS);
    }

    public static Command getCollectFromNeutralZoneCommand() {
        return new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        getSafeDriveToPoseCommand(isRight() ? () -> FieldConstants.RIGHT_INTAKE_POSITION : () -> FieldConstants.LEFT_INTAKE_POSITION),
                        new GamePieceAutoDriveCommand().withTimeout(AutonomousConstants.NEUTRAL_ZONE_COLLECTION_TIMEOUT_SECONDS).finallyDo(() -> System.out.println("ENDED"))
                ),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.INTAKE),
                ShootingCommands.getShootAtHubCommand().onlyWhile(AutonomousCommands::isInAllianceZone)
        );
    }

    public static Command getScoreCommand() {
        return new ParallelCommandGroup(
                getSafeDriveToPoseCommand(() -> isRight() ? FieldConstants.RIGHT_IDEAL_SHOOTING_POSITION : FieldConstants.LEFT_IDEAL_SHOOTING_POSITION),
                ShootingCommands.getShootAtHubCommand().onlyWhile(AutonomousCommands::isInAllianceZone).repeatedly()
        ).withTimeout(AutonomousConstants.SCORING_TIMEOUT_SECONDS);
    }

    public static Command getCollectFromDepotCommand() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        getSafeDriveToPoseCommand(() -> FieldConstants.DEPOT_POSITION),
                        ShootingCommands.getShootAtHubCommand().onlyWhile(AutonomousCommands::isInAllianceZone).repeatedly()
                ).until(() -> RobotContainer.SWERVE.atPose(FieldConstants.DEPOT_POSITION)),
                new GamePieceAutoDriveCommand().alongWith(ShootingCommands.getShootAtHubCommand())
        ).alongWith(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.INTAKE)).withTimeout(AutonomousConstants.DEPOT_COLLECTION_TIMEOUT_SECONDS);
    }

    public static Command getClimbCommand(FlippablePose2d climbPosition) {
        return new SequentialCommandGroup(
                getSafeDriveToPoseCommand(() -> climbPosition),
                new InstantCommand() // TODO: Add climb command
        );
    }

    private static Command getSafeDriveToPoseCommand(Supplier<FlippablePose2d> targetPose) {
        return new ConditionalCommand(
                getDriveThroughTrenchCommand(targetPose),
                SwerveCommands.getDriveToPoseCommand(targetPose, AutonomousConstants.DRIVE_IN_AUTONOMOUS_CONSTRAINTS),
                () -> shouldDriveThroughTrench(targetPose.get())
        );
    }

    private static Command getDriveThroughTrenchCommand(Supplier<FlippablePose2d> targetPose) {
        return new SequentialCommandGroup(
                SwerveCommands.getDriveToPoseCommand(AutonomousCommands::getTrenchEntryPose, AutonomousConstants.DRIVE_IN_AUTONOMOUS_CONSTRAINTS, 4),
                SwerveCommands.getDriveToPoseCommand(AutonomousCommands::getTrenchExitPose, AutonomousConstants.DRIVE_IN_AUTONOMOUS_CONSTRAINTS, 4),
                SwerveCommands.getDriveToPoseCommand(targetPose, AutonomousConstants.DRIVE_IN_AUTONOMOUS_CONSTRAINTS)
        );
    }

    private static boolean shouldDriveThroughTrench(FlippablePose2d targetPose) {
        return (!isInAllianceZone() && isPoseInAllianceZone(targetPose)) || (isInAllianceZone() && !isPoseInAllianceZone(targetPose));
    }


    private static FlippablePose2d getTrenchExitPose() {
        final FlippablePose2d targetTrenchExitPose = isRight() ?
                isInAllianceZone() ? FieldConstants.RIGHT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE : FieldConstants.RIGHT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE :
                isInAllianceZone() ? FieldConstants.LEFT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE : FieldConstants.LEFT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE;
        System.out.println("Target Trench Exit Pose: " + targetTrenchExitPose.get().toString());
        if (RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getRotation().getDegrees() > 90)
            return new FlippablePose2d(targetTrenchExitPose.getBlueObject().getTranslation(), Math.PI, true);
        return targetTrenchExitPose;
    }

    private static FlippablePose2d getTrenchEntryPose() {
        final FlippablePose2d targetTrenchEntryPose = isRight() ?
                isInAllianceZone() ? FieldConstants.RIGHT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE : FieldConstants.RIGHT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE :
                isInAllianceZone() ? FieldConstants.LEFT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE : FieldConstants.LEFT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE;
        if (RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getRotation().getDegrees() > 90)
            return new FlippablePose2d(targetTrenchEntryPose.getBlueObject().getTranslation(), Math.PI, true);
        return targetTrenchEntryPose;
    }

    private static boolean isRight() {
        return RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation().getY() < FieldConstants.FIELD_WIDTH_METERS / 2;
    }

    private static boolean isInAllianceZone() {
        return isPoseInAllianceZone(new FlippablePose2d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose(), true));
    }

    private static boolean isPoseInAllianceZone(FlippablePose2d pose) {
        return pose.get().getX() < FieldConstants.ALLIANCE_ZONE_LENGTH;
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