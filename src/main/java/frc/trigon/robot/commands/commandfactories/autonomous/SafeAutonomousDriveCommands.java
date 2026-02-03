package frc.trigon.robot.commands.commandfactories.autonomous;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.lib.utilities.flippable.FlippableRotation2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class SafeAutonomousDriveCommands {
    public static Command getSafeDriveToPoseCommand(Supplier<FlippablePose2d> targetPose, PathConstraints normalPathConstrains) {
        return getSafeDriveToPoseCommand(targetPose, normalPathConstrains, 0);
    }

    public static Command getSafeDriveToPoseCommand(Supplier<FlippablePose2d> targetPose, PathConstraints normalPathConstrains, double endVelocity) {
        return getSafeDriveToPoseCommand(targetPose, normalPathConstrains, endVelocity, null, 0);
    }

    public static Command getSafeDriveToPoseCommand(
            Supplier<FlippablePose2d> targetPose,
            PathConstraints normalPathConstrains,
            double endVelocity,
            PathConstraints driveSlowlyInAllianceZoneConstraints,
            double driveSlowlyInAllianceZoneTime
    ) {
        return new ConditionalCommand(
                getDriveThroughTrenchCommand(targetPose, normalPathConstrains, endVelocity, driveSlowlyInAllianceZoneConstraints, driveSlowlyInAllianceZoneTime),
                getDriveSlowlyInAllianceZoneCommand(targetPose, AutonomousConstants.DRIVE_IN_AUTONOMOUS_CONSTRAINTS, endVelocity, driveSlowlyInAllianceZoneConstraints, driveSlowlyInAllianceZoneTime),
                () -> shouldDriveThroughTrench(targetPose.get())
        ).alongWith(new RunCommand(() -> Logger.recordOutput("Autonomous/ShouldDriveThroughTrench", shouldDriveThroughTrench(targetPose.get()))));
    }

    private static Command getDriveThroughTrenchCommand(
            Supplier<FlippablePose2d> targetPose,
            PathConstraints normalPathConstrains,
            double endVelocity,
            PathConstraints driveSlowlyInAllianceZoneConstraints,
            double driveSlowlyInAllianceZoneTime
    ) {
        return new SequentialCommandGroup(
                getDriveSlowlyInAllianceZoneCommand(() -> getTrenchEntryPose(targetPose.get()), normalPathConstrains, 4, driveSlowlyInAllianceZoneConstraints, driveSlowlyInAllianceZoneTime),
                getDriveSlowlyInAllianceZoneCommand(() -> getTrenchExitPose(targetPose.get()), normalPathConstrains, 4, driveSlowlyInAllianceZoneConstraints, driveSlowlyInAllianceZoneTime),
                getDriveSlowlyInAllianceZoneCommand(targetPose, normalPathConstrains, endVelocity, driveSlowlyInAllianceZoneConstraints, driveSlowlyInAllianceZoneTime)
        );
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
        final FlippablePose2d targetTrenchExitPose = isRight() ?
                getClosestPoseToPose(targetPose, FieldConstants.RIGHT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE, FieldConstants.RIGHT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE) :
                getClosestPoseToPose(targetPose, FieldConstants.LEFT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE, FieldConstants.LEFT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE);
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