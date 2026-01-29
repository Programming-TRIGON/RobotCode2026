package frc.trigon.robot.commands.commandfactories;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
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
                AutonomousConstants.FIRST_AUTONOMOUS_CHOOSER.get() == null ? new InstantCommand() : AutonomousConstants.FIRST_AUTONOMOUS_CHOOSER.get(),
                AutonomousConstants.SECOND_AUTONOMOUS_CHOOSER.get() == null ? new InstantCommand() : AutonomousConstants.SECOND_AUTONOMOUS_CHOOSER.get(),
                AutonomousConstants.THIRD_AUTONOMOUS_CHOOSER.get() == null ? new InstantCommand() : AutonomousConstants.THIRD_AUTONOMOUS_CHOOSER.get(),
                getClimbCommand(AutonomousConstants.CLIMB_POSITION_CHOOSER.get()).onlyIf(() -> AutonomousConstants.CLIMB_POSITION_CHOOSER.get() != null)
        );
    }

    public static Command getDeliveryCommand() {
        return new ParallelCommandGroup(
                SwerveCommands.getDriveToPoseCommand(isRight() ? () -> FieldConstants.RIGHT_INTAKE_POSITION : () -> FieldConstants.LEFT_INTAKE_POSITION, AutonomousConstants.DRIVE_IN_AUTONOMOUS_CONSTRAINTS),
                FuelIntakeCommands.getIntakeCommand(),
                GeneralCommands.getContinuousConditionalCommand(
                        ShootingCommands.getShootAtHubCommand(),
                        ShootingCommands.getDeliveryCommand(),
                        AutonomousCommands::isInAllianceZone
                )
        ).withTimeout(AutonomousConstants.DELIVERY_TIMEOUT_SECONDS);
    }

    public static Command getCollectFromNeutralZoneCommand() {
        return new ParallelCommandGroup(
                SwerveCommands.getDriveToPoseCommand(isRight() ? () -> FieldConstants.RIGHT_INTAKE_POSITION : () -> FieldConstants.LEFT_INTAKE_POSITION, AutonomousConstants.DRIVE_IN_AUTONOMOUS_CONSTRAINTS),
                FuelIntakeCommands.getIntakeCommand(),
                ShootingCommands.getShootAtHubCommand().onlyIf(AutonomousCommands::isInAllianceZone)
        ).withTimeout(AutonomousConstants.NEUTRAL_ZONE_COLLECTION_TIMEOUT_SECONDS);
    }

    public static Command getScoreCommand() {
        return new ParallelCommandGroup(
                SwerveCommands.getDriveToPoseCommand(isRight() ? () -> FieldConstants.RIGHT_IDEAL_SHOOTING_POSITION : () -> FieldConstants.LEFT_IDEAL_SHOOTING_POSITION, AutonomousConstants.DRIVE_IN_AUTONOMOUS_CONSTRAINTS),
                ShootingCommands.getShootAtHubCommand().onlyIf(AutonomousCommands::isInAllianceZone)
        );
    }

    public static Command getCollectFromDepotCommand() {
        return new ParallelCommandGroup(
                SwerveCommands.getDriveToPoseCommand(() -> FieldConstants.DEPOT_POSITION, AutonomousConstants.DRIVE_IN_AUTONOMOUS_CONSTRAINTS),
                FuelIntakeCommands.getIntakeCommand(),
                ShootingCommands.getShootAtHubCommand().onlyIf(AutonomousCommands::isInAllianceZone)
        ).withTimeout(AutonomousConstants.DEPOT_COLLECTION_TIMEOUT_SECONDS);
    }

    public static Command getClimbCommand(FlippablePose2d climbPosition) {
        return new SequentialCommandGroup(
                SwerveCommands.getDriveToPoseCommand(() -> climbPosition, AutonomousConstants.DRIVE_IN_AUTONOMOUS_CONSTRAINTS),
                null // TODO: Add climb command
        );
    }

    private static boolean isRight() {
        return RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation().getY() < FieldConstants.FIELD_WIDTH_METERS / 2;
    }

    private static boolean isInAllianceZone() {
        final Pose2d currentRobotPose = new FlippablePose2d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose(), true).get();
        return currentRobotPose.getX() < FieldConstants.ALLIANCE_ZONE_LENGTH;
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