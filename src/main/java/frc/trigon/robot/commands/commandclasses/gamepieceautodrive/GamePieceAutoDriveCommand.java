package frc.trigon.robot.commands.commandclasses.gamepieceautodrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.utilities.flippable.FlippableRotation2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.misc.objectdetection.ObjectPoseEstimator;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class GamePieceAutoDriveCommand extends ParallelCommandGroup {
    private static final ObjectPoseEstimator OBJECT_POSE_ESTIMATOR = RobotContainer.OBJECT_POSE_ESTIMATOR;
    private final AtomicReference<GamePieceCluster> currentTargetCluster = new AtomicReference<>();

    public GamePieceAutoDriveCommand() {
        addCommands(
                createTargetUpdateCommand(),
                createDriveCommand()
        );
    }

    /**
     * Creates a command that periodically calculates the best cluster and updates the atomic reference.
     * This acts as the "vision processing" thread for this command group.
     *
     * @return A {@link Command} that runs continuously to update the target.
     */
    private Command createTargetUpdateCommand() {
        return new RunCommand(() -> {
            GamePieceCluster bestCluster = findBestCluster();
            currentTargetCluster.set(bestCluster);
        });
    }

    /**
     * Creates the drive command that moves the robot towards the currently selected target cluster.
     *
     * @return A {@link Command} that executes the swerve drive logic.
     */
    private Command createDriveCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(AutonomousConstants.GAME_PIECE_AUTO_DRIVE_X_PID_CONTROLLER::reset),
                SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                        this::getXControllerOutput,
                        this::getYControllerOutput,
                        this::getTargetHeading
                )
        ).onlyWhile(this::hasValidTarget);
    }

    /**
     * Calculates the output for the X-axis PID controller based on the current target.
     *
     * @return The calculated output for the X-axis velocity, or 0.0 if no target exists.
     */
    private double getXControllerOutput() {
        Translation2d error = getTranslationError();
        if (error == null)
            return 0.0;
        return AutonomousConstants.GAME_PIECE_AUTO_DRIVE_X_PID_CONTROLLER.calculate(error.getX());
    }

    /**
     * Calculates the output for the Y-axis PID controller based on the current target.
     *
     * @return The calculated output for the Y-axis velocity, or 0.0 if no target exists.
     */
    private double getYControllerOutput() {
        Translation2d error = getTranslationError();
        if (error == null) return 0.0;
        return AutonomousConstants.GAME_PIECE_AUTO_DRIVE_Y_PID_CONTROLLER.calculate(error.getY());
    }

    /**
     * Retrieves the target heading for the robot based on the selected cluster's approach angle.
     *
     * @return A {@link FlippableRotation2d} representing the desired heading, or null if no target exists.
     */
    private FlippableRotation2d getTargetHeading() {
        GamePieceCluster cluster = currentTargetCluster.get();
        if (cluster == null)
            return null;
        return new FlippableRotation2d(cluster.getApproachHeading(), false);
    }

    /**
     * Calculates the vector from the robot to the target centroid, rotated to be robot-relative.
     * This is used as the error input for the self-relative PID controllers.
     *
     * @return A {@link Translation2d} representing the distance error in robot-relative coordinates, or null if no target exists.
     */
    private Translation2d getTranslationError() {
        GamePieceCluster cluster = currentTargetCluster.get();
        if (cluster == null)
            return null;

        Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        Translation2d fieldRelativeError = cluster.getCentroid().minus(robotPose.getTranslation());

        return fieldRelativeError.rotateBy(robotPose.getRotation().unaryMinus());
    }

    /**
     * Checks if a valid target is currently identified and if the robot is far enough away to continue driving.
     *
     * @return {@code true} if a target exists and distance is greater than the intake check distance; {@code false} otherwise.
     */
    private boolean hasValidTarget() {
        Translation2d error = getTranslationError();
        return error != null && error.getNorm() > AutonomousConstants.AUTO_COLLECTION_INTAKE_OPEN_CHECK_DISTANCE_METERS;
    }

    /**
     * Scans all tracked objects on the field, groups them into clusters, and identifies the "best" one to target.
     * The selection is based on a scoring algorithm that weighs cluster size against distance from the robot.
     *
     * @return The {@link GamePieceCluster} with the highest score, or null if no valid game pieces are found.
     */
    private GamePieceCluster findBestCluster() {
        List<Translation2d> allObjects = OBJECT_POSE_ESTIMATOR.getObjectsOnField();
        Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();

        if (allObjects.isEmpty())
            return null;

        GamePieceCluster bestCluster = null;
        double maxScore = -Double.MAX_VALUE;

        for (Translation2d seed : allObjects) {
            if (isOutOfBounds(seed)) continue;

            List<Translation2d> clusterPieces = getNeighbors(seed, allObjects);
            if (clusterPieces.isEmpty()) continue;

            GamePieceCluster cluster = new GamePieceCluster(clusterPieces, robotPose);
            double score = calculateClusterScore(clusterPieces.size(), cluster.getDistanceToRobot());

            if (score > maxScore) {
                maxScore = score;
                bestCluster = cluster;
            }
        }
        return bestCluster;
    }

    /**
     * Finds all game pieces that are within a specific radius of a seed piece to form a cluster.
     *
     * @param seed       The central game piece used to find neighbors.
     * @param allObjects A list of all detected game pieces on the field.
     * @return A list of {@link Translation2d} objects representing the cluster, including the seed.
     */
    private List<Translation2d> getNeighbors(Translation2d seed, List<Translation2d> allObjects) {
        List<Translation2d> neighbors = new ArrayList<>();
        for (Translation2d other : allObjects) {
            if (isOutOfBounds(other)) continue;

            if (seed.getDistance(other) <= GamePieceAutoDriveConstants.CLUSTER_RADIUS_METERS)
                neighbors.add(other);
        }
        return neighbors;
    }

    /**
     * Checks if a game piece is outside the valid collection area (e.g., on the opponent's side of the field).
     *
     * @param piece The position of the game piece to check.
     * @return {@code true} if the piece is out of bounds; {@code false} otherwise.
     */
    private boolean isOutOfBounds(Translation2d piece) {
        return piece.getX() > GamePieceAutoDriveConstants.MAX_COLLECTION_X_METERS;
    }

    /**
     * Calculates a score for a potential target cluster.
     * Higher scores are better.
     *
     * @param count    The number of game pieces in the cluster.
     * @param distance The distance from the robot to the cluster centroid in meters.
     * @return The calculated score.
     */
    private double calculateClusterScore(int count, double distance) {
        return (count * GamePieceAutoDriveConstants.SCORE_WEIGHT_COUNT) - (distance * GamePieceAutoDriveConstants.SCORE_PENALTY_DISTANCE);
    }
}