package frc.trigon.robot.constants;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.trigon.lib.utilities.FilesHandler;
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

    public static final double LEFT_TRENCH_Y_POSITION_METERS = 7.48;
    public static final FlippablePose2d
            LEFT_CLIMB_POSITION = new FlippablePose2d(1.57, 4.25, Rotation2d.fromDegrees(0), true),
            RIGHT_CLIMB_POSITION = new FlippablePose2d(LEFT_CLIMB_POSITION.getBlueObject().getX(), 3.28, Rotation2d.fromDegrees(0), true),
            CENTER_CLIMB_POSITION = new FlippablePose2d((LEFT_CLIMB_POSITION.getBlueObject().getX() + RIGHT_CLIMB_POSITION.getBlueObject().getX()) / 2, LEFT_CLIMB_POSITION.getBlueObject().getY(), Rotation2d.fromDegrees(0), true),
            DEPOT_POSITION = new FlippablePose2d(0.45, 7, Rotation2d.fromDegrees(-90), true),
            LEFT_INTAKE_POSITION = new FlippablePose2d(7.4, 7.3, Rotation2d.fromDegrees(-90), true),
            RIGHT_INTAKE_POSITION = new FlippablePose2d(LEFT_INTAKE_POSITION.getBlueObject().getX(), FIELD_WIDTH_METERS - LEFT_INTAKE_POSITION.getBlueObject().getY(), Rotation2d.fromDegrees(90), true),
            LEFT_START_INTAKING_FOR_DELIVERY_POSITION = new FlippablePose2d(LEFT_INTAKE_POSITION.getBlueObject().getX(), LEFT_INTAKE_POSITION.getBlueObject().getY(), Rotation2d.fromDegrees(-100), true),
            RIGHT_START_INTAKING_FOR_DELIVERY_POSITION = new FlippablePose2d(LEFT_START_INTAKING_FOR_DELIVERY_POSITION.getBlueObject().getX(), RIGHT_INTAKE_POSITION.getBlueObject().getY(), Rotation2d.fromDegrees(100), true),
            LEFT_IDEAL_SHOOTING_POSITION = new FlippablePose2d(2.7, 5.8, Rotation2d.fromDegrees(0), true),
            RIGHT_IDEAL_SHOOTING_POSITION = new FlippablePose2d(LEFT_IDEAL_SHOOTING_POSITION.getBlueObject().getX(), FIELD_WIDTH_METERS - LEFT_IDEAL_SHOOTING_POSITION.getBlueObject().getY(), Rotation2d.fromDegrees(0), true),
            LEFT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE = new FlippablePose2d(3.8, LEFT_TRENCH_Y_POSITION_METERS, Rotation2d.kZero, true),
            RIGHT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE = new FlippablePose2d(LEFT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE.getBlueObject().getX(), FIELD_WIDTH_METERS - LEFT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE.getBlueObject().getY(), Rotation2d.kZero, true),
            LEFT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE = new FlippablePose2d(5.6, LEFT_TRENCH_Y_POSITION_METERS, Rotation2d.kZero, true),
            RIGHT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE = new FlippablePose2d(LEFT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE.getBlueObject().getX(), FIELD_WIDTH_METERS - LEFT_TRENCH_Y_POSITION_METERS, Rotation2d.kZero, true);
    private static final double
            BLUE_RELATIVE_DELIVERY_POSITION_X = 3,
            DELIVERY_POSITION_Y_OFFSET_FROM_CENTER_METERS = 2.2;
    public static final FlippableTranslation2d
            HUB_POSITION = new FlippableTranslation2d(TAG_ID_TO_POSE.get(26).getX() + (Units.inchesToMeters(47) / 2), FIELD_WIDTH_METERS / 2, true),
            RIGHT_DELIVERY_POSITION = new FlippableTranslation2d(BLUE_RELATIVE_DELIVERY_POSITION_X, (FIELD_WIDTH_METERS / 2) - DELIVERY_POSITION_Y_OFFSET_FROM_CENTER_METERS, true),
            LEFT_DELIVERY_POSITION = new FlippableTranslation2d(BLUE_RELATIVE_DELIVERY_POSITION_X, (FIELD_WIDTH_METERS / 2) + DELIVERY_POSITION_Y_OFFSET_FROM_CENTER_METERS, true);
    public static final double
            ALLIANCE_ZONE_LENGTH = 4.5,
            DELIVERY_ZONE_START_BLUE_X = ALLIANCE_ZONE_LENGTH + 1;

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

    public static boolean isInAllianceZone() {
        final Pose2d currentRobotPose = new FlippablePose2d(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose(), true).get();
        return currentRobotPose.getX() < ALLIANCE_ZONE_LENGTH;
    }
}
