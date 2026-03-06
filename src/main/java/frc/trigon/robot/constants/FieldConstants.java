package frc.trigon.robot.constants;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
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
            FIELD_WIDTH_METERS = FlippingUtil.fieldSizeY,
            FIELD_LENGTH_METERS = FlippingUtil.fieldSizeX;

    private static final List<Integer> I_HATE_YOU = List.of(
            1, 6, 7, 12, 13, 14, 15, 16, 17, 22, 23, 28, 29, 30, 31, 32
    );
    private static final boolean SHOULD_USE_HOME_TAG_LAYOUT = false;
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = createAprilTagFieldLayout();
    private static final Transform3d TAG_OFFSET = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    public static final HashMap<Integer, Pose3d> TAG_ID_TO_POSE = fieldLayoutToTagIDToPoseMap();

    public static final double LEFT_TRENCH_Y_POSITION_METERS = 7.4;
    private static final double
            CLIMB_X = 1.57,
            LEFT_CLIMB_Y = 4.25,
            RIGHT_CLIMB_Y = 3.28,
            DEPOT_X = 0.45,
            DEPOT_Y = 7.0,
            INTAKE_X = 7.4,
            INTAKE_Y = 7.3,
            IDEAL_SHOOTING_X = 2.7,
            IDEAL_SHOOTING_Y = 5.8,
            TRENCH_ALLIANCE_X = 3.9,
            TRENCH_NEUTRAL_X = 5.53,
            TRENCH_ENTRY_Y = 7.4,
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
            LEFT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE = new FlippablePose2d(TRENCH_ALLIANCE_X, TRENCH_ENTRY_Y, Rotation2d.kZero, true),
            RIGHT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE = mirror(LEFT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE),
            LEFT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE = new FlippablePose2d(TRENCH_NEUTRAL_X, TRENCH_ENTRY_Y, Rotation2d.kZero, true),
            RIGHT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE = mirror(LEFT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE);
    public static final FlippableTranslation2d
            HUB_POSITION = new FlippableTranslation2d(4.7, FIELD_WIDTH_METERS / 2, true),
            RIGHT_DELIVERY_POSITION = new FlippableTranslation2d(BLUE_RELATIVE_DELIVERY_POSITION_X, (FIELD_WIDTH_METERS / 2) - DELIVERY_POSITION_Y_OFFSET_FROM_CENTER_METERS, true),
            LEFT_DELIVERY_POSITION = new FlippableTranslation2d(BLUE_RELATIVE_DELIVERY_POSITION_X, (FIELD_WIDTH_METERS / 2) + DELIVERY_POSITION_Y_OFFSET_FROM_CENTER_METERS, true);
    public static final double
            ALLIANCE_ZONE_LENGTH = 4.5,
            DELIVERY_ZONE_START_BLUE_X = ALLIANCE_ZONE_LENGTH + 1,
            LEFT_TRENCH_MIN_Y = 7.2,
            RIGHT_TRENCH_MAX_Y = FIELD_WIDTH_METERS - LEFT_TRENCH_MIN_Y;

    private static AprilTagFieldLayout createAprilTagFieldLayout() {
        try {
            return SHOULD_USE_HOME_TAG_LAYOUT ?
                    new AprilTagFieldLayout(FilesHandler.DEPLOY_PATH + "field_calibration.json") :
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
        Logger.recordOutput("Zones/IsRobotInTrenchZone", isRobotInTrenchZone());
        Logger.recordOutput("Zones/IsRobotInTrenchXRange", isRobotInTrenchXRange());
        Logger.recordOutput("Zones/IsRobotInTrenchYRange", isRobotInTrenchYRange());
        Logger.recordOutput("Zones/IsRobotInDeliveryZone", isRobotInDeliveryZone());
        Logger.recordOutput("Zones/IsRobotInAllianceZone", isRobotInAllianceZone());
    }

    public static boolean isRobotInTrenchZone() {
        return isPoseInTrenchZone(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation());
    }

    public static boolean isPoseInTrenchZone(Translation2d pose) {
        if (pose == null)
            return false;
        return isPoseInTrenchXRange(pose) && isPoseInTrenchYRange(pose);
    }

    public static boolean isRobotInTrenchXRange() {
        return isPoseInTrenchXRange(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation());
    }

    public static boolean isPoseInTrenchXRange(Translation2d pose) {
        if (pose == null)
            return false;
        return (pose.getX() > TRENCH_ALLIANCE_X && pose.getX() < TRENCH_NEUTRAL_X) || (pose.getX() > FIELD_LENGTH_METERS - TRENCH_NEUTRAL_X && pose.getX() < FIELD_LENGTH_METERS - TRENCH_ALLIANCE_X);
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