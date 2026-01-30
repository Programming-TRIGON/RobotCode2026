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

    private Command createTargetUpdateCommand() {
        return new RunCommand(() -> {
            GamePieceCluster bestCluster = findBestCluster();
            currentTargetCluster.set(bestCluster);
        });
    }

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

    private double getXControllerOutput() {
        Translation2d error = getTranslationError();
        if (error == null) return 0.0;

        // CRITICAL FIX 1: Negate the error here.
        // Controller calculates (Setpoint - Measurement).
        // We want (0 - (-Distance)) = +Distance to drive forward.
        return AutonomousConstants.GAME_PIECE_AUTO_DRIVE_X_PID_CONTROLLER.calculate(-error.getX());
    }

    private double getYControllerOutput() {
        Translation2d error = getTranslationError();
        if (error == null) return 0.0;

        // CRITICAL FIX 1: Negate the error here too.
        return AutonomousConstants.GAME_PIECE_AUTO_DRIVE_Y_PID_CONTROLLER.calculate(-error.getY());
    }

    private FlippableRotation2d getTargetHeading() {
        GamePieceCluster cluster = currentTargetCluster.get();
        if (cluster == null) return null;
        return new FlippableRotation2d(cluster.getApproachHeading(), false);
    }

    private Translation2d getTranslationError() {
        GamePieceCluster cluster = currentTargetCluster.get();
        if (cluster == null) return null;

        Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        Translation2d fieldRelativeError = cluster.getCentroid().minus(robotPose.getTranslation());

        // CRITICAL FIX 2: You MUST use unaryMinus() here.
        // This rotates the "World Vector" backwards to match the "Robot's Perspective".
        // Without this, Left/Right are swapped, causing swerving.
        return fieldRelativeError.rotateBy(robotPose.getRotation().unaryMinus());
    }

    private boolean hasValidTarget() {
        Translation2d error = getTranslationError();
        Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();

        // CRITICAL FIX 3: Stop if the ROBOT is past the line.
        // Previously we only checked if the GAME PIECE was past the line.
        boolean robotIsSafe = robotPose.getX() < GamePieceAutoDriveConstants.MAX_COLLECTION_X_METERS;

        return error != null &&
                robotIsSafe &&
                error.getNorm() > AutonomousConstants.AUTO_COLLECTION_INTAKE_OPEN_CHECK_DISTANCE_METERS;
    }

    // --- Vision Logic (Unchanged) ---
    private GamePieceCluster findBestCluster() {
        List<Translation2d> allObjects = OBJECT_POSE_ESTIMATOR.getObjectsOnField();
        Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();

        if (allObjects.isEmpty()) return null;

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

    private List<Translation2d> getNeighbors(Translation2d seed, List<Translation2d> allObjects) {
        List<Translation2d> neighbors = new ArrayList<>();
        for (Translation2d other : allObjects) {
            if (isOutOfBounds(other)) continue;
            if (seed.getDistance(other) <= GamePieceAutoDriveConstants.CLUSTER_RADIUS_METERS)
                neighbors.add(other);
        }
        return neighbors;
    }

    private boolean isOutOfBounds(Translation2d piece) {
        return piece.getX() > GamePieceAutoDriveConstants.MAX_COLLECTION_X_METERS;
    }

    private double calculateClusterScore(int count, double distance) {
        return (count * GamePieceAutoDriveConstants.SCORE_WEIGHT_COUNT) - (distance * GamePieceAutoDriveConstants.SCORE_PENALTY_DISTANCE);
    }
}