package frc.trigon.robot.constants;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.trigon.lib.utilities.FilesHandler;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.lib.utilities.flippable.FlippableTranslation2d;
import frc.trigon.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;

public class FieldConstants {
    public static final double
            FIELD_WIDTH_METERS = 8.069326,
            FIELD_LENGTH_METERS = 16.540988;

    private static final List<Integer> I_HATE_YOU = List.of(
            1, 6, 7, 12, 13, 14, 15, 16, 17, 22, 23, 28, 29, 30, 31, 32
    );
    private static final boolean SHOULD_USE_HOME_TAG_LAYOUT = false;
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = createAprilTagFieldLayout();
    private static final Transform3d TAG_OFFSET = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    public static final HashMap<Integer, Pose3d> TAG_ID_TO_POSE = fieldLayoutToTagIDToPoseMap();

    public static final double
            CLIMB_X = 1.57,
            LEFT_CLIMB_Y = 4.25,
            RIGHT_CLIMB_Y = 3.28,
            DEPOT_X = 0.45,
            DEPOT_Y = 7.0,
            INTAKE_X = 7,
            INTAKE_Y = 4,
            IDEAL_SHOOTING_X = 3.85, // 4.3 - need fix!
            IDEAL_SHOOTING_Y = 7.48,
            TRENCH_ALLIANCE_ENTRY_AUTONOMOUS_X = 3.85,
            TRENCH_NEUTRAL_ENTRY_AUTONOMOUS_X = 5.58,
            TRENCH_ENTRY_Y = 7.48,
            BLUE_RELATIVE_DELIVERY_POSITION_X = 3.0,
            DELIVERY_POSITION_Y_OFFSET_FROM_CENTER_METERS = 2.2;

    public static final FlippablePose2d
            LEFT_CLIMB_POSITION = new FlippablePose2d(CLIMB_X, LEFT_CLIMB_Y, Rotation2d.kZero, true),
            RIGHT_CLIMB_POSITION = new FlippablePose2d(CLIMB_X, RIGHT_CLIMB_Y, Rotation2d.kZero, true),
            CENTER_CLIMB_POSITION = new FlippablePose2d(CLIMB_X, LEFT_CLIMB_Y, Rotation2d.kZero, true),
            DEPOT_POSITION = new FlippablePose2d(DEPOT_X, DEPOT_Y, Rotation2d.fromDegrees(-90), true),
            LEFT_INTAKE_POSITION = new FlippablePose2d(INTAKE_X, INTAKE_Y, Rotation2d.fromDegrees(-90), true),
            RIGHT_INTAKE_POSITION = mirror(LEFT_INTAKE_POSITION),
            LEFT_START_INTAKING_FOR_DELIVERY_POSITION = new FlippablePose2d(FIELD_LENGTH_METERS / 2.0, INTAKE_Y, Rotation2d.fromDegrees(-100), true),
            RIGHT_START_INTAKING_FOR_DELIVERY_POSITION = mirror(LEFT_START_INTAKING_FOR_DELIVERY_POSITION),
            LEFT_IDEAL_SHOOTING_POSITION = new FlippablePose2d(IDEAL_SHOOTING_X, IDEAL_SHOOTING_Y, Rotation2d.kZero, true),
            RIGHT_IDEAL_SHOOTING_POSITION = mirror(LEFT_IDEAL_SHOOTING_POSITION),
            LEFT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE = new FlippablePose2d(TRENCH_ALLIANCE_ENTRY_AUTONOMOUS_X, TRENCH_ENTRY_Y, Rotation2d.kZero, true),
            RIGHT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE = mirror(LEFT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE),
            LEFT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE = new FlippablePose2d(TRENCH_NEUTRAL_ENTRY_AUTONOMOUS_X, TRENCH_ENTRY_Y, Rotation2d.kZero, true),
            RIGHT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE = mirror(LEFT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE);
    public static final FlippableTranslation2d
            HUB_POSITION = new FlippableTranslation2d(TAG_ID_TO_POSE.get(26).getX() + (Units.inchesToMeters(47) / 2), FIELD_WIDTH_METERS / 2, true),
            RIGHT_DELIVERY_POSITION = new FlippableTranslation2d(BLUE_RELATIVE_DELIVERY_POSITION_X, (FIELD_WIDTH_METERS / 2) - DELIVERY_POSITION_Y_OFFSET_FROM_CENTER_METERS, true),
            LEFT_DELIVERY_POSITION = new FlippableTranslation2d(BLUE_RELATIVE_DELIVERY_POSITION_X, (FIELD_WIDTH_METERS / 2) + DELIVERY_POSITION_Y_OFFSET_FROM_CENTER_METERS, true);
    public static final double
            ALLIANCE_ZONE_LENGTH = 4.5,
            DELIVERY_ZONE_START_BLUE_X = ALLIANCE_ZONE_LENGTH + 0.93,
            TRENCH_ZONE_MINIMUM_X = 4.4,
            TRENCH_ZONE_MAXIMUM_X = 4.9,
            LEFT_TRENCH_MIN_Y = 6.9,
            RIGHT_TRENCH_MAX_Y = FIELD_WIDTH_METERS - LEFT_TRENCH_MIN_Y;
    private static final double TRENCH_POSE_PREDICTION_TIME_SECONDS = 0.23;

    static {
        Logger.recordOutput("IMPORTANT/HUB_POSE_BLUE", HUB_POSITION.getBlueObject());
        Logger.recordOutput("IMPORTANT/HUB_POSE_RED", FlippingUtil.flipFieldPosition(HUB_POSITION.getBlueObject()));
        Logger.recordOutput("IMPORTANT/TAG_Y", TAG_ID_TO_POSE.get(26).getY());
    }

    private static AprilTagFieldLayout createAprilTagFieldLayout() {
        try {
            return SHOULD_USE_HOME_TAG_LAYOUT ?
                    new AprilTagFieldLayout(FilesHandler.DEPLOY_PATH + "2026-frc-welded-home.json") :
                    AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private static HashMap<Integer, Pose3d> fieldLayoutToTagIDToPoseMap() {
        final HashMap<Integer, Pose3d> tagIDToPose = new HashMap<>();
        for (AprilTag aprilTag : APRIL_TAG_FIELD_LAYOUT.getTags())
            if (!I_HATE_YOU.contains(aprilTag.ID))
                tagIDToPose.put(aprilTag.ID, aprilTag.pose.transformBy(TAG_OFFSET));

        return tagIDToPose;
    }

    /**
     * Mirrors a FlippablePose2d across the field's Y-axis centerline.
     */
    public static FlippablePose2d mirror(FlippablePose2d pose) {
        final Pose2d basePose = pose.getBlueObject();
        return new FlippablePose2d(
                basePose.getX(),
                FIELD_WIDTH_METERS - basePose.getY(),
                Rotation2d.fromDegrees(-basePose.getRotation().getDegrees()),
                true
        );
    }

    public static void logZoneChecks() {
        Logger.recordOutput("Zones/IsRobotInDeliveryZone", isRobotInDeliveryZone());
        Logger.recordOutput("Zones/IsRobotInAllianceZone", isRobotInAllianceZone());
        Logger.recordOutput("Zones/IsPassingThroughTrenchZone", isPassingThroughTrenchZone());
    }

    public static boolean isPassingThroughTrenchZone() {
        return FieldConstants.isTurretInTrenchZone() ||
                FieldConstants.willTurretBeInTrenchZone(TRENCH_POSE_PREDICTION_TIME_SECONDS) ||
                (FieldConstants.isTurretBeforeTrench() && FieldConstants.willTurretBeAfterTrench(TRENCH_POSE_PREDICTION_TIME_SECONDS)) ||
                (FieldConstants.isTurretAfterTrench() && FieldConstants.willTurretBeBeforeTrench(TRENCH_POSE_PREDICTION_TIME_SECONDS));
    }

    public static boolean willTurretBeInTrenchZone(double predictionSeconds) {
        return isPoseInTrenchZone(RobotContainer.TURRET.getPredictedTurretFieldRelativePosition(predictionSeconds).getTranslation());
    }

    public static boolean isTurretInTrenchZone() {
        return isPoseInTrenchZone(RobotContainer.TURRET.getCurrentTurretFieldRelativePosition().getTranslation());
    }

    public static boolean isPoseInTrenchZone(Translation2d pose) {
        if (pose == null)
            return false;
        return isPoseInTrenchXRange(pose) && isPoseInTrenchYRange(pose);
    }

    public static boolean isPoseInTrenchXRange(Translation2d pose) {
        if (pose == null)
            return false;
        return (pose.getX() > TRENCH_ZONE_MINIMUM_X && pose.getX() < TRENCH_ZONE_MAXIMUM_X) || (pose.getX() > FIELD_LENGTH_METERS - TRENCH_ZONE_MAXIMUM_X && pose.getX() < FIELD_LENGTH_METERS - TRENCH_ZONE_MINIMUM_X);
    }

    public static boolean willTurretBeBeforeTrench(double predictionSeconds) {
        return isPoseBeforeTrench(RobotContainer.TURRET.getPredictedTurretFieldRelativePosition(predictionSeconds).getTranslation());
    }

    public static boolean isTurretBeforeTrench() {
        return isPoseBeforeTrench(RobotContainer.TURRET.getCurrentTurretFieldRelativePosition().getTranslation());
    }

    public static boolean isPoseBeforeTrench(Translation2d pose) {
        if (pose == null)
            return false;
        return pose.getX() < TRENCH_ZONE_MINIMUM_X || pose.getX() > FIELD_LENGTH_METERS - TRENCH_ZONE_MINIMUM_X;
    }

    public static boolean willTurretBeAfterTrench(double predictionSeconds) {
        return isPoseAfterTrench(RobotContainer.TURRET.getPredictedTurretFieldRelativePosition(predictionSeconds).getTranslation());
    }

    public static boolean isTurretAfterTrench() {
        return isPoseAfterTrench(RobotContainer.TURRET.getCurrentTurretFieldRelativePosition().getTranslation());
    }

    public static boolean isPoseAfterTrench(Translation2d pose) {
        if (pose == null)
            return false;
        return pose.getX() > TRENCH_ZONE_MAXIMUM_X && pose.getX() < FIELD_LENGTH_METERS - TRENCH_ZONE_MAXIMUM_X;
    }

    public static boolean isRobotInTrenchYRange() {
        return isPoseInTrenchYRange(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation());
    }

    public static boolean isPoseInTrenchYRange(Translation2d pose) {
        if (pose == null)
            return false;
        return pose.getY() > FieldConstants.LEFT_TRENCH_MIN_Y || pose.getY() < FieldConstants.RIGHT_TRENCH_MAX_Y;
    }

    public static boolean isRobotInDeliveryZone() {
        return isPoseInDeliveryZone(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation());
    }

    public static boolean isPoseInDeliveryZone(Translation2d pose) {
        if (pose == null)
            return false;
        if (Flippable.isRedAlliance())
            return pose.getX() < FieldConstants.FIELD_LENGTH_METERS - FieldConstants.DELIVERY_ZONE_START_BLUE_X;
        return pose.getX() > FieldConstants.DELIVERY_ZONE_START_BLUE_X;
    }

    public static boolean isRobotInAllianceZone() {
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