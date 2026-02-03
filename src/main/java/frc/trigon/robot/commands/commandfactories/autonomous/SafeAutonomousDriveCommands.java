package frc.trigon.robot.commands.commandfactories.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.littletonrobotics.junction.Logger;

import java.util.List;
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
            PathConstraints driveSlowlyInAllianceZoneConstraints, double driveSlowlyInAllianceZoneTime
    ) {
        return new ConditionalCommand(
                getDriveThroughTrenchCommand(targetPose, normalPathConstrains, endVelocity, driveSlowlyInAllianceZoneConstraints, driveSlowlyInAllianceZoneTime),
                getDriveSlowlyInAllianceZoneCommand(targetPose, AutonomousConstants.DRIVE_IN_AUTONOMOUS_CONSTRAINTS, endVelocity, driveSlowlyInAllianceZoneConstraints, driveSlowlyInAllianceZoneTime),
                () -> shouldDriveThroughTrench(targetPose.get())
        ).alongWith(new RunCommand(() -> Logger.recordOutput("Autonomous/ShouldDriveThroughTrench", shouldDriveThroughTrench(targetPose.get()))));
    }

    private static Command getDriveThroughTrenchCommand(
            Supplier<FlippablePose2d> targetPose, PathConstraints normalPathConstrains, double endVelocity,
            PathConstraints driveSlowlyInAllianceZoneConstraints, double driveSlowlyInAllianceZoneTime
    ) {
        return new ConditionalCommand(
                getPathThroughTrenchCommand(targetPose.get(), normalPathConstrains, endVelocity),
                getDriveThroughTrenchWithDrivingSlowlyCommand(
                        targetPose.get(),
                        normalPathConstrains,
                        endVelocity,
                        driveSlowlyInAllianceZoneConstraints,
                        driveSlowlyInAllianceZoneTime
                ),
                () -> driveSlowlyInAllianceZoneTime != 0
        );
    }

    private static Command getDriveThroughTrenchWithDrivingSlowlyCommand(
            FlippablePose2d targetPose, PathConstraints normalPathConstrains, double endVelocity,
            PathConstraints driveSlowlyInAllianceZoneConstraints, double driveSlowlyInAllianceZoneTime
    ) {
        return new SequentialCommandGroup(
                getPathThroughTrenchCommand(targetPose, normalPathConstrains, endVelocity).until(SafeAutonomousDriveCommands::isInAllianceZone),
                getPathThroughTrenchCommand(targetPose, driveSlowlyInAllianceZoneConstraints, endVelocity).withTimeout(driveSlowlyInAllianceZoneTime).onlyWhile(SafeAutonomousDriveCommands::isInAllianceZone),
                getPathThroughTrenchCommand(targetPose, normalPathConstrains, endVelocity)
        );
    }

    private static Command getPathThroughTrenchCommand(
            FlippablePose2d targetPose,
            PathConstraints normalPathConstrains,
            double endVelocity
    ) {
        return AutoBuilder.followPath(getPathThroughTrench(
                targetPose,
                normalPathConstrains,
                endVelocity
        ));
    }

    private static PathPlannerPath getPathThroughTrench(FlippablePose2d targetPose, PathConstraints normalPathConstrains, double endVelocity) {
        final List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose(),
                getTrenchEntryPose(targetPose).get(),
                getTrenchExitPose(targetPose).get(),
                targetPose.get()
        );

        final PathPlannerPath path = new PathPlannerPath(
                waypoints,
                normalPathConstrains,
                null,
                new GoalEndState(endVelocity, targetPose.get().getRotation())
        );

        path.preventFlipping = true;
        return path;
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

    private static boolean shouldDriveThroughTrench(FlippablePose2d targetPose) {
        return (!isInAllianceZone() && isPoseInAllianceZone(targetPose.get())) || (isInAllianceZone() && !isPoseInAllianceZone(targetPose.get()));
    }

    private static FlippablePose2d getTrenchExitPose(FlippablePose2d targetPose) {
        final FlippablePose2d targetTrenchExitPose = isInAllianceZone() ?
                getLowestTravelDistancePose(targetPose, FieldConstants.RIGHT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE, FieldConstants.LEFT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE) :
                getLowestTravelDistancePose(targetPose, FieldConstants.RIGHT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE, FieldConstants.LEFT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE);
        if (targetPose.get().getRotation().getDegrees() > 90 || targetPose.get().getRotation().getDegrees() < -90)
            return new FlippablePose2d(targetTrenchExitPose.get().getTranslation(), RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getRotation().getRadians(), false);
        return targetTrenchExitPose;
    }

    private static FlippablePose2d getTrenchEntryPose(FlippablePose2d targetPose) {
        final FlippablePose2d targetTrenchEntryPose = isInAllianceZone() ?
                getLowestTravelDistancePose(targetPose, FieldConstants.RIGHT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE, FieldConstants.LEFT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE) :
                getLowestTravelDistancePose(targetPose, FieldConstants.RIGHT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE, FieldConstants.LEFT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE);
        if (targetPose.get().getRotation().getDegrees() > 90 || targetPose.get().getRotation().getDegrees() < -90)
            return new FlippablePose2d(targetTrenchEntryPose.getBlueObject().getTranslation(), 0, true);
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

    private static FlippablePose2d getClosestPoseToPose(FlippablePose2d pose, FlippablePose2d... poses) {
        FlippablePose2d closestPose = null;
        double closestDistance = Double.MAX_VALUE;
        for (FlippablePose2d candidatePose : poses) {
            final double distance = pose.get().getTranslation().getDistance(candidatePose.get().getTranslation());
            if (distance < closestDistance) {
                closestDistance = distance;
                closestPose = candidatePose;
            }
        }
        return closestPose;
    }

    public static boolean isRight() {
        if (Flippable.isRedAlliance())
            return RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation().getY() > FieldConstants.FIELD_WIDTH_METERS / 2;
        return RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation().getY() < FieldConstants.FIELD_WIDTH_METERS / 2;
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