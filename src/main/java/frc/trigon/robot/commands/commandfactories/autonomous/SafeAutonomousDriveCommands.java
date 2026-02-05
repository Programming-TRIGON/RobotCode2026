package frc.trigon.robot.commands.commandfactories.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.utilities.Conversions;
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
            PathConstraints driveSlowlyInAllianceZoneConstraints, double driveSlowlyInAllianceZonePercentage
    ) {
        return new ConditionalCommand(
                getDriveThroughTrenchCommand(
                        targetPose.get(),
                        normalPathConstrains,
                        endVelocity,
                        driveSlowlyInAllianceZoneConstraints,
                        driveSlowlyInAllianceZonePercentage
                ),
                getDriveSlowlyInAllianceZoneCommand(
                        targetPose,
                        normalPathConstrains,
                        endVelocity,
                        driveSlowlyInAllianceZoneConstraints,
                        driveSlowlyInAllianceZonePercentage
                ),
                () -> shouldDriveThroughTrench(targetPose.get())
        ).raceWith(new RunCommand(() -> Logger.recordOutput("Autonomous/ShouldDriveThroughTrench", shouldDriveThroughTrench(targetPose.get()))));
    }

    private static Command getDriveThroughTrenchCommand(
            FlippablePose2d targetPose, PathConstraints normalPathConstrains, double endVelocity,
            PathConstraints driveSlowlyInAllianceZoneConstraints, double driveSlowlyInAllianceZonePercentage
    ) {
        return new DeferredCommand(
                () -> AutoBuilder.followPath(getPathThroughTrench(
                                targetPose,
                                normalPathConstrains,
                                endVelocity,
                                driveSlowlyInAllianceZoneConstraints,
                                driveSlowlyInAllianceZonePercentage
                        )
                ),
                Set.of(RobotContainer.SWERVE)
        );
    }

    private static PathPlannerPath getPathThroughTrench(
            FlippablePose2d targetPose, PathConstraints normalPathConstrains, double endVelocity,
            PathConstraints driveSlowlyInAllianceZoneConstraints, double driveInAllianceZonePercentage
    ) {
        final Pose2d currentRobotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final Pose2d trenchEntryPose = getTrenchEntryPose(targetPose).get();
        final Pose2d trenchExitPose = getTrenchExitPose(targetPose).get();
        final List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(currentRobotPose.getTranslation(), trenchEntryPose.getTranslation().minus(currentRobotPose.getTranslation()).getAngle()),
                trenchEntryPose,
                new Pose2d(trenchExitPose.getTranslation(), targetPose.get().getTranslation().minus(trenchExitPose.getTranslation()).getAngle()),
                new Pose2d(targetPose.get().getTranslation(), targetPose.get().getTranslation().minus(trenchExitPose.getTranslation()).getAngle())
        );

        final Rotation2d targetTrenchDrivingHolonomicAngle = getTrenchDrivingHolonomicAngle(targetPose);
        final RotationTarget trenchEntryRotationTarget = new RotationTarget(
                1,
                targetTrenchDrivingHolonomicAngle
        );
        final RotationTarget trenchExitRotationTarget = new RotationTarget(
                2,
                targetTrenchDrivingHolonomicAngle
        );
        final RotationTarget finalRotationTarget = new RotationTarget(
                3,
                targetPose.get().getRotation()
        );
        final ConstraintsZone driveSlowlyInAllianceZoneConstraintsZone = new ConstraintsZone(
                isInAllianceZone() ? 0 : 1.5,
                isInAllianceZone() ?
                        getDriveSlowlyPathPosition(
                                driveInAllianceZonePercentage,
                                currentRobotPose.getTranslation().getDistance(trenchEntryPose.getTranslation()),
                                trenchEntryPose.getTranslation().getDistance(trenchExitPose.getTranslation()) / 2) :
                        Math.min(2 + getDriveSlowlyPathPosition(
                                driveInAllianceZonePercentage,
                                trenchEntryPose.getTranslation().getDistance(trenchExitPose.getTranslation()) / 2,
                                trenchExitPose.getTranslation().getDistance(targetPose.get().getTranslation())), 3),
                driveSlowlyInAllianceZoneConstraints
        );

        final PathPlannerPath path = new PathPlannerPath(
                waypoints,
                List.of(trenchEntryRotationTarget, trenchExitRotationTarget, finalRotationTarget),
                List.of(),
                List.of(driveSlowlyInAllianceZoneConstraintsZone),
                List.of(),
                normalPathConstrains,
                null,
                new GoalEndState(endVelocity, targetPose.get().getRotation()),
                false
        );

        path.preventFlipping = true;
        return path;
    }

    private static Rotation2d getTrenchDrivingHolonomicAngle(FlippablePose2d targetPose) {
        if (targetPose.get().getRotation().getDegrees() > 90 || targetPose.get().getRotation().getDegrees() < -90)
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

    private static double getDriveSlowlyPathPosition(double percentage, double distanceFromInitToEntry, double distanceFromEntryToCenter) {
        final double distanceFromInitToCenter = distanceFromInitToEntry + distanceFromEntryToCenter;
        final double d = percentage * distanceFromInitToCenter;
        double a = Math.min(d / distanceFromInitToEntry, 1);
        double b = Math.max((d - distanceFromInitToEntry) / distanceFromEntryToCenter, 0);
        return a + b;
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