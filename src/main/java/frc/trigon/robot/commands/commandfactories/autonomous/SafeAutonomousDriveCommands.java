package frc.trigon.robot.commands.commandfactories.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

public class SafeAutonomousDriveCommands {
    public static Command getSafeDriveToPoseCommand(Supplier<FlippablePose2d> targetPose, PathConstraints normalPathConstrains) {
        return getSafeDriveToPoseCommand(targetPose, normalPathConstrains, 0);
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
        ).raceWith(new RunCommand(() -> Logger.recordOutput("Autonomous/ShouldDriveThroughTrench", shouldDriveThroughTrench(targetPose.get()))));
    }

    private static Command getDriveThroughTrenchCommand(FlippablePose2d targetPose, PathConstraints normalPathConstrains, double endVelocity, PathConstraints driveSlowlyInAllianceZoneConstraints, double driveSlowlyInAllianceZoneTimeSeconds) {
        return new ConditionalCommand(
                getDriveThroughTrenchFromAllianceZoneCommand(targetPose, normalPathConstrains, endVelocity, driveSlowlyInAllianceZoneConstraints, driveSlowlyInAllianceZoneTimeSeconds),
                getDriveThroughTrenchFromNeutralZoneCommand(targetPose, normalPathConstrains, endVelocity, driveSlowlyInAllianceZoneConstraints, driveSlowlyInAllianceZoneTimeSeconds),
                SafeAutonomousDriveCommands::isInAllianceZone
        );
    }

    private static Command getDriveThroughTrenchFromAllianceZoneCommand(FlippablePose2d targetPose, PathConstraints normalPathConstrains, double endVelocity, PathConstraints driveSlowlyInAllianceZoneConstraints, double driveSlowlyInAllianceZoneTimeSeconds) {
        return new SequentialCommandGroup(
                getDriveThroughTrenchCommand(targetPose, driveSlowlyInAllianceZoneConstraints, endVelocity).withTimeout(driveSlowlyInAllianceZoneTimeSeconds).onlyWhile(SafeAutonomousDriveCommands::isInAllianceZone),
                getDriveThroughTrenchCommand(targetPose, normalPathConstrains, endVelocity)
        );
    }

    private static Command getDriveThroughTrenchFromNeutralZoneCommand(FlippablePose2d targetPose, PathConstraints normalPathConstrains, double endVelocity, PathConstraints driveSlowlyInAllianceZoneConstraints, double driveSlowlyInAllianceZoneTimeSeconds) {
        return new SequentialCommandGroup(
                getDriveThroughTrenchCommand(targetPose, normalPathConstrains, normalPathConstrains.maxVelocityMPS()).until(SafeAutonomousDriveCommands::isInAllianceZone),
                getDriveThroughTrenchCommand(targetPose, driveSlowlyInAllianceZoneConstraints, endVelocity).withTimeout(driveSlowlyInAllianceZoneTimeSeconds).onlyWhile(SafeAutonomousDriveCommands::isInAllianceZone),
                getDriveSlowlyInAllianceZoneCommand(() -> targetPose, normalPathConstrains, endVelocity, driveSlowlyInAllianceZoneConstraints, driveSlowlyInAllianceZoneTimeSeconds).onlyWhile(SafeAutonomousDriveCommands::isInAllianceZone)
        );
    }

    private static Command getDriveThroughTrenchCommand(FlippablePose2d targetPose, PathConstraints normalPathConstrains, double endVelocity) {
        return new DeferredCommand(
                () -> AutoBuilder.followPath(getPathThroughTrench(
                                targetPose,
                                normalPathConstrains,
                                endVelocity
                        )
                ),
                Set.of(RobotContainer.SWERVE)
        );
    }

    private static int num = 0;

    private static PathPlannerPath getPathThroughTrench(FlippablePose2d targetPose, PathConstraints pathConstraints, double endVelocity) {
        num++;
        Logger.recordOutput("Autonomous/numPaths", num);
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
        // 1. Calculate the general direction of travel (Heading)
        // This is the angle from the robot's current position to the final target.
        Rotation2d travelHeading = targetPose.getTranslation()
                .minus(currentRobotPose.getTranslation())
                .getAngle();

        if (
                (currentRobotPose.getX() > trenchEntryPose.getX() && isInAllianceZone() && !Flippable.isRedAlliance()) ||
                        (currentRobotPose.getTranslation().getX() < trenchEntryPose.getX() && !isInAllianceZone() && !Flippable.isRedAlliance()) ||
                        (currentRobotPose.getX() > trenchEntryPose.getX() && !isInAllianceZone() && Flippable.isRedAlliance()) ||
                        (currentRobotPose.getX() < trenchEntryPose.getX() && isInAllianceZone() && Flippable.isRedAlliance())
        ) {
            final Pose2d closestPoseToTarget = getClosestPoseToPose(targetPose, trenchEntryPose, trenchExitPose);

            return PathPlannerPath.waypointsFromPoses(
                    new Pose2d(currentRobotPose.getTranslation(), closestPoseToTarget.getTranslation().minus(currentRobotPose.getTranslation()).getAngle()),

                    // --- FIX STARTS HERE ---
                    // Instead of using closestPoseToTarget.getRotation() (which is static)
                    // or isSecondPath (which is a patch), use the dynamic travelHeading.
                    new Pose2d(closestPoseToTarget.getTranslation(), travelHeading),
                    // --- FIX ENDS HERE ---

                    new Pose2d(targetPose.getTranslation(), targetPose.getTranslation().minus(trenchExitPose.getTranslation()).getAngle())
            );
        }

        return PathPlannerPath.waypointsFromPoses(
                new Pose2d(currentRobotPose.getTranslation(), trenchEntryPose.getTranslation().minus(currentRobotPose.getTranslation()).getAngle()),

                // Apply the same logic here if 'trenchEntryPose' has a fixed rotation that might conflict
                new Pose2d(trenchEntryPose.getTranslation(), travelHeading),
                new Pose2d(trenchExitPose.getTranslation(), travelHeading),

                new Pose2d(targetPose.getTranslation(), targetPose.getTranslation().minus(trenchExitPose.getTranslation()).getAngle())
        );
    }

    private static List<RotationTarget> getRotationTargetsThroughTrench(FlippablePose2d targetPose, Pose2d currentPose, List<Waypoint> waypoints) {
        final Rotation2d targetTrenchDrivingHolonomicAngle = getTrenchDrivingHolonomicAngle(currentPose);
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

    private static Rotation2d getTrenchDrivingHolonomicAngle(Pose2d currentRobotPose) {
        if (currentRobotPose.getRotation().getDegrees() > 90 || currentRobotPose.getRotation().getDegrees() < -90)
            return Rotation2d.k180deg;
        return Rotation2d.kZero;
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
        FlippablePose2d lowestTravelDistancePose = null;
        double lowestTravelDistance = Double.MAX_VALUE;
        for (FlippablePose2d candidatePose : poses) {
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

    public static boolean isRight() {
        if (Flippable.isRedAlliance())
            return RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation().getY() > FieldConstants.FIELD_WIDTH_METERS / 2;
        return RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation().getY() < FieldConstants.FIELD_WIDTH_METERS / 2;
    }

    private static boolean shouldDriveThroughTrench(FlippablePose2d targetPose) {
        return (!isInAllianceZone() && isPoseInAllianceZone(targetPose.get())) || (isInAllianceZone() && !isPoseInAllianceZone(targetPose.get()));
    }

    public static boolean isInAllianceZone() {
        return isPoseInAllianceZone(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose());
    }

    private static boolean isPoseInAllianceZone(Pose2d pose) {
        if (Flippable.isRedAlliance())
            return pose.getX() > FieldConstants.FIELD_LENGTH_METERS - FieldConstants.ALLIANCE_ZONE_LENGTH;
        return pose.getX() < FieldConstants.ALLIANCE_ZONE_LENGTH;
    }
}