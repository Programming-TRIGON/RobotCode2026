package frc.trigon.robot.commands.commandfactories.autonomous;

import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.function.Supplier;

public class SafeAutonomousDriveCommands {
    public static Command getSafeDriveToPoseCommand(Supplier<FlippablePose2d> targetPose, PathConstraints normalPathConstraints) {
        return getSafeDriveToPoseCommand(targetPose, normalPathConstraints, 0);
    }

    public static Command getSafeDriveToPoseCommand(Supplier<FlippablePose2d> targetPose, PathConstraints normalPathConstrains, double endVelocity) {
        return getSafeDriveToPoseCommand(targetPose, normalPathConstrains, endVelocity, null, 0);
    }

    public static Command getSafeDriveToPoseCommand(
            Supplier<FlippablePose2d> targetPose, PathConstraints normalPathConstrains, double endVelocity,
            PathConstraints driveSlowlyInAllianceZoneConstraints, double driveSlowlyInAllianceZoneTimeSeconds
    ) {
        return new ConditionalCommand(
                getDriveThroughTrenchCommand(
                        targetPose.get(),
                        normalPathConstrains,
                        endVelocity,
                        driveSlowlyInAllianceZoneConstraints,
                        driveSlowlyInAllianceZoneTimeSeconds
                ),
                getDriveSlowlyInAllianceZoneCommand(
                        targetPose,
                        normalPathConstrains,
                        endVelocity,
                        driveSlowlyInAllianceZoneConstraints,
                        driveSlowlyInAllianceZoneTimeSeconds
                ),
                () -> shouldDriveThroughTrench(targetPose.get())
        ).raceWith(new RunCommand(() -> Logger.recordOutput("Autonomous/ShouldDriveThroughTrench", shouldDriveThroughTrench(targetPose.get())))).onlyIf(() -> targetPose.get() != null);
    }

    private static Command getDriveThroughTrenchCommand(FlippablePose2d targetPose, PathConstraints normalPathConstrains, double endVelocity, PathConstraints driveSlowlyInAllianceZoneConstraints, double driveSlowlyInAllianceZoneTimeSeconds) {
        return new ConditionalCommand(
                getDriveThroughTrenchFromAllianceZoneCommand(targetPose, normalPathConstrains, endVelocity, driveSlowlyInAllianceZoneConstraints, driveSlowlyInAllianceZoneTimeSeconds),
                getDriveThroughTrenchFromNeutralZoneCommand(targetPose, normalPathConstrains, endVelocity, driveSlowlyInAllianceZoneConstraints, driveSlowlyInAllianceZoneTimeSeconds),
                SafeAutonomousDriveCommands::isInAllianceZone
        ).onlyIf(() -> targetPose != null);
    }

    private static Command getDriveThroughTrenchFromAllianceZoneCommand(FlippablePose2d targetPose, PathConstraints normalPathConstrains, double endVelocity, PathConstraints driveSlowlyInAllianceZoneConstraints, double driveSlowlyInAllianceZoneTimeSeconds) {
        return new SequentialCommandGroup(
                getDriveThroughTrenchCommand(targetPose, driveSlowlyInAllianceZoneConstraints, endVelocity).withTimeout(driveSlowlyInAllianceZoneTimeSeconds).onlyWhile(SafeAutonomousDriveCommands::isInAllianceZone),
                getDriveThroughTrenchCommand(targetPose, normalPathConstrains, endVelocity)
        ).onlyIf(() -> targetPose != null);
    }

    private static Command getDriveThroughTrenchFromNeutralZoneCommand(FlippablePose2d targetPose, PathConstraints normalPathConstrains, double endVelocity, PathConstraints driveSlowlyInAllianceZoneConstraints, double driveSlowlyInAllianceZoneTimeSeconds) {
        return new SequentialCommandGroup(
                getDriveThroughTrenchCommand(targetPose, normalPathConstrains, normalPathConstrains.maxVelocityMPS()).until(SafeAutonomousDriveCommands::isInAllianceZone),
                getDriveThroughTrenchCommand(targetPose, driveSlowlyInAllianceZoneConstraints, endVelocity).withTimeout(driveSlowlyInAllianceZoneTimeSeconds).onlyWhile(SafeAutonomousDriveCommands::isInAllianceZone),
                getDriveSlowlyInAllianceZoneCommand(() -> targetPose, normalPathConstrains, endVelocity, driveSlowlyInAllianceZoneConstraints, driveSlowlyInAllianceZoneTimeSeconds).onlyWhile(SafeAutonomousDriveCommands::isInAllianceZone)
        ).onlyIf(() -> targetPose != null);
    }

    private static Command getDriveThroughTrenchCommand(FlippablePose2d targetPose, PathConstraints normalPathConstrains, double endVelocity) {
        return SwerveCommands.getFollowPathCommand(() -> getPathThroughTrench(targetPose, normalPathConstrains, endVelocity)).onlyIf(() -> targetPose != null);
    }

    private static Command getDriveSlowlyInAllianceZoneCommand(
            Supplier<FlippablePose2d> targetPose,
            PathConstraints normalPathConstrains,
            double endVelocity,
            PathConstraints driveSlowlyInAllianceZoneConstraints,
            double driveSlowlyInAllianceZoneTime
    ) {
        if (driveSlowlyInAllianceZoneTime == 0)
            return SwerveCommands.getDriveToPoseCommand(targetPose, normalPathConstrains, endVelocity);

        return new SequentialCommandGroup(
                SwerveCommands.getDriveToPoseCommand(targetPose, normalPathConstrains, endVelocity).until(SafeAutonomousDriveCommands::isInAllianceZone),
                SwerveCommands.getDriveToPoseCommand(targetPose, driveSlowlyInAllianceZoneConstraints, endVelocity).withTimeout(driveSlowlyInAllianceZoneTime).onlyWhile(SafeAutonomousDriveCommands::isInAllianceZone),
                SwerveCommands.getDriveToPoseCommand(targetPose, normalPathConstrains, endVelocity)
        );
    }

    private static PathPlannerPath getPathThroughTrench(FlippablePose2d targetPose, PathConstraints pathConstraints, double endVelocity) {
        if (targetPose == null)
            return new PathPlannerPath(
                    PathPlannerPath.waypointsFromPoses(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose(), RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose()),
                    pathConstraints,
                    null,
                    new GoalEndState(endVelocity, RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getRotation())
            );

        final Pose2d currentRobotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final Pose2d trenchEntryPose = getTrenchEntryPose(targetPose).get();
        final Pose2d trenchExitPose = getTrenchExitPose(targetPose).get();
        final List<Waypoint> waypoints = getWaypointsThroughTrench(
                currentRobotPose,
                trenchEntryPose,
                trenchExitPose,
                targetPose.get()
        );
        final List<RotationTarget> rotationTargets = getRotationTargetsThroughTrench(targetPose, currentRobotPose, waypoints);

        final PathPlannerPath path = new PathPlannerPath(
                waypoints,
                rotationTargets,
                List.of(),
                List.of(),
                List.of(),
                pathConstraints,
                null,
                new GoalEndState(endVelocity, targetPose.get().getRotation()),
                false
        );

        path.preventFlipping = true;
        return path;
    }

    private static List<Waypoint> getWaypointsThroughTrench(Pose2d currentRobotPose, Pose2d trenchEntryPose, Pose2d trenchExitPose, Pose2d targetPose) {
        Rotation2d travelHeading = targetPose.getTranslation().minus(currentRobotPose.getTranslation()).getAngle();

        if ((currentRobotPose.getX() > trenchEntryPose.getX() && isInAllianceZone() && !Flippable.isRedAlliance()) ||
                (currentRobotPose.getTranslation().getX() < trenchEntryPose.getX() && !isInAllianceZone() && !Flippable.isRedAlliance()) ||
                (currentRobotPose.getX() > trenchEntryPose.getX() && !isInAllianceZone() && Flippable.isRedAlliance()) ||
                (currentRobotPose.getX() < trenchEntryPose.getX() && isInAllianceZone() && Flippable.isRedAlliance())) {
            final Pose2d closestPoseToTarget = getClosestPoseToPose(targetPose, trenchEntryPose, trenchExitPose);

            return PathPlannerPath.waypointsFromPoses(
                    new Pose2d(currentRobotPose.getTranslation(), closestPoseToTarget.getTranslation().minus(currentRobotPose.getTranslation()).getAngle()),
                    new Pose2d(closestPoseToTarget.getTranslation(), getHeading(travelHeading)),
                    new Pose2d(targetPose.getTranslation(), targetPose.getTranslation().minus(trenchExitPose.getTranslation()).getAngle())
            );
        }

        return PathPlannerPath.waypointsFromPoses(
                new Pose2d(currentRobotPose.getTranslation(), trenchEntryPose.getTranslation().minus(currentRobotPose.getTranslation()).getAngle()),
                new Pose2d(trenchEntryPose.getTranslation(), getHeading(travelHeading)),
                new Pose2d(trenchExitPose.getTranslation(), getHeading(travelHeading)),
                new Pose2d(targetPose.getTranslation(), targetPose.getTranslation().minus(trenchExitPose.getTranslation()).getAngle())
        );
    }

    private static List<RotationTarget> getRotationTargetsThroughTrench(FlippablePose2d targetPose, Pose2d currentPose, List<Waypoint> waypoints) {
        final Rotation2d targetTrenchDrivingHolonomicAngle = getHeading(shouldRotateBeforeTrench(targetPose, currentPose) ? targetPose.get().getRotation() : currentPose.getRotation());
        if (waypoints.size() == 3) {
            return List.of(
                    new RotationTarget(1, targetTrenchDrivingHolonomicAngle),
                    new RotationTarget(1.7, targetPose.get().getRotation()),
                    new RotationTarget(2, targetPose.get().getRotation())
            );
        }
        return List.of(
                new RotationTarget(1, targetTrenchDrivingHolonomicAngle),
                new RotationTarget(2, targetTrenchDrivingHolonomicAngle),
                new RotationTarget(2.7, targetPose.get().getRotation()),
                new RotationTarget(3, targetPose.get().getRotation())
        );
    }

    private static Rotation2d getHeading(Rotation2d travelHeading) {
        double lowestAngleDifference = Double.MAX_VALUE;
        Rotation2d bestHeading = travelHeading;
        List<Integer> possibleHeadingsDegrees = List.of(0, 180, -180);
        for (int possibleHeadingDegrees : possibleHeadingsDegrees) {
            Rotation2d possibleHeading = Rotation2d.fromDegrees(possibleHeadingDegrees);
            double angleDifference = Math.abs(travelHeading.minus(possibleHeading).getDegrees());
            if (angleDifference < lowestAngleDifference) {
                lowestAngleDifference = angleDifference;
                bestHeading = possibleHeading;
            }
        }
        return bestHeading;
    }

    private static FlippablePose2d getTrenchExitPose(FlippablePose2d targetPose) {
        final FlippablePose2d targetTrenchExitPose = isInAllianceZone() ?
                getLowestTravelDistancePose(targetPose, FieldConstants.RIGHT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE, FieldConstants.LEFT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE) :
                getLowestTravelDistancePose(targetPose, FieldConstants.RIGHT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE, FieldConstants.LEFT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE);
        if (!isInAllianceZone())
            return new FlippablePose2d(targetTrenchExitPose.getBlueObject().getTranslation(), Rotation2d.kPi.getRadians(), true);
        return targetTrenchExitPose;
    }

    private static FlippablePose2d getTrenchEntryPose(FlippablePose2d targetPose) {
        final FlippablePose2d targetTrenchEntryPose = isInAllianceZone() ?
                getLowestTravelDistancePose(targetPose, FieldConstants.RIGHT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE, FieldConstants.LEFT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE) :
                getLowestTravelDistancePose(targetPose, FieldConstants.RIGHT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE, FieldConstants.LEFT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE);
        if (!isInAllianceZone())
            return new FlippablePose2d(targetTrenchEntryPose.getBlueObject().getTranslation(), Rotation2d.kPi.getRadians(), true);
        return targetTrenchEntryPose;
    }

    private static FlippablePose2d getLowestTravelDistancePose(FlippablePose2d targetPose, FlippablePose2d... poses) {
        if (targetPose == null)
            return null;
        FlippablePose2d lowestTravelDistancePose = null;
        double lowestTravelDistance = Double.MAX_VALUE;
        for (FlippablePose2d candidatePose : poses) {
            if (candidatePose == null)
                continue;
            final double distanceFromRobot = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation().getDistance(candidatePose.get().getTranslation());
            final double distanceToTarget = candidatePose.get().getTranslation().getDistance(targetPose.get().getTranslation());
            final double travelDistance = distanceToTarget + distanceFromRobot;
            if (travelDistance < lowestTravelDistance) {
                lowestTravelDistance = travelDistance;
                lowestTravelDistancePose = candidatePose;
            }
        }
        return lowestTravelDistancePose;
    }

    private static Pose2d getClosestPoseToPose(Pose2d pose, Pose2d... poses) {
        Pose2d closestPose = null;
        double closestDistance = Double.MAX_VALUE;
        for (Pose2d candidatePose : poses) {
            final double distance = pose.getTranslation().getDistance(candidatePose.getTranslation());
            if (distance < closestDistance) {
                closestDistance = distance;
                closestPose = candidatePose;
            }
        }
        return closestPose;
    }

    private static boolean shouldRotateBeforeTrench(FlippablePose2d targetPose, Pose2d currentPose) {
        if (targetPose == null || currentPose == null)
            return false;

        final Pose2d trenchEntry = getTrenchEntryPose(targetPose).get();
        final Pose2d trenchExit = getTrenchExitPose(targetPose).get();

        final double distanceBeforeTrench = currentPose.getTranslation().getDistance(trenchEntry.getTranslation());
        final double distanceAfterTrench = targetPose.get().getTranslation().getDistance(trenchExit.getTranslation());

        return distanceBeforeTrench > distanceAfterTrench;
    }

    public static boolean isRight() {
        if (Flippable.isRedAlliance())
            return RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation().getY() > FieldConstants.FIELD_WIDTH_METERS / 2;
        return RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation().getY() < FieldConstants.FIELD_WIDTH_METERS / 2;
    }

    private static boolean shouldDriveThroughTrench(FlippablePose2d targetPose) {
        if (targetPose == null)
            return false;
        return (!isInAllianceZone() && isPoseInAllianceZone(targetPose.get().getTranslation())) || (isInAllianceZone() && !isPoseInAllianceZone(targetPose.get().getTranslation()));
    }

    public static boolean isInAllianceZone() {
        return isPoseInAllianceZone(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation());
    }

    public static boolean isPoseInAllianceZone(Translation2d pose) {
        if (pose == null)
            return false;
        if (Flippable.isRedAlliance())
            return pose.getX() > FieldConstants.FIELD_LENGTH_METERS - FieldConstants.ALLIANCE_ZONE_LENGTH;
        return pose.getX() < FieldConstants.ALLIANCE_ZONE_LENGTH;
    }
}