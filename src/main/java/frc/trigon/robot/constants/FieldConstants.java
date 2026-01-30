package frc.trigon.robot.constants;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.lib.utilities.FilesHandler;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.lib.utilities.flippable.FlippableTranslation2d;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;

public class FieldConstants {
    public static final double
            FIELD_WIDTH_METERS = FlippingUtil.fieldSizeY,
            FIELD_LENGTH_METERS = FlippingUtil.fieldSizeX;

    private static final List<Integer> I_HATE_YOU = List.of(
            //Tags to ignore
    );
    private static final boolean SHOULD_USE_HOME_TAG_LAYOUT = false;
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = createAprilTagFieldLayout();
    private static final Transform3d TAG_OFFSET = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    public static final HashMap<Integer, Pose3d> TAG_ID_TO_POSE = fieldLayoutToTagIDToPoseMap();

    public static final FlippablePose2d
            LEFT_CLIMB_POSITION = new FlippablePose2d(1.45, 4.25, Rotation2d.fromDegrees(0), true),
            RIGHT_CLIMB_POSITION = new FlippablePose2d(LEFT_CLIMB_POSITION.getBlueObject().getX(), 3.28, Rotation2d.fromDegrees(0), true),
            CENTER_CLIMB_POSITION = new FlippablePose2d((LEFT_CLIMB_POSITION.getBlueObject().getX() + LEFT_CLIMB_POSITION.getBlueObject().getX()) / 2, LEFT_CLIMB_POSITION.getBlueObject().getY(), Rotation2d.fromDegrees(0), true),
            DEPOT_POSITION = new FlippablePose2d(0.9, 6, Rotation2d.fromDegrees(180), true),
            LEFT_INTAKE_POSITION = new FlippablePose2d(7, 7.3, Rotation2d.fromDegrees(-40), true),
            RIGHT_INTAKE_POSITION = new FlippablePose2d(LEFT_INTAKE_POSITION.getBlueObject().getX(), FIELD_WIDTH_METERS - LEFT_INTAKE_POSITION.getBlueObject().getY(), Rotation2d.fromDegrees(40), true),
            LEFT_IDEAL_SHOOTING_POSITION = new FlippablePose2d(2.7, 5.8, Rotation2d.fromDegrees(0), true),
            RIGHT_IDEAL_SHOOTING_POSITION = new FlippablePose2d(LEFT_IDEAL_SHOOTING_POSITION.getBlueObject().getX(), FIELD_WIDTH_METERS - LEFT_IDEAL_SHOOTING_POSITION.getBlueObject().getY(), Rotation2d.fromDegrees(0), true),
            LEFT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE = new FlippablePose2d(3.9, 7.4, Rotation2d.kZero, true),
            RIGHT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE = new FlippablePose2d(LEFT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE.getBlueObject().getX(), FIELD_WIDTH_METERS - LEFT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE.getBlueObject().getY(), Rotation2d.kZero, true),
            LEFT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE = new FlippablePose2d(5.53, LEFT_TRENCH_ENTRY_POSITION_FROM_ALLIANCE_ZONE.getBlueObject().getY(), Rotation2d.kZero, true),
            RIGHT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE = new FlippablePose2d(LEFT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE.getBlueObject().getX(), FIELD_WIDTH_METERS - LEFT_TRENCH_ENTRY_POSITION_FROM_NEUTRAL_ZONE.getBlueObject().getY(), Rotation2d.kZero, true);
    private static final double
            BLUE_RELATIVE_DELIVERY_POSITION_X = 3,
            DELIVERY_POSITION_Y_OFFSET_FROM_CENTER_METERS = 2.2;
    public static final FlippableTranslation2d
            HUB_POSITION = new FlippableTranslation2d(4.7, FIELD_WIDTH_METERS / 2, true),
            RIGHT_DELIVERY_POSITION = new FlippableTranslation2d(BLUE_RELATIVE_DELIVERY_POSITION_X, (FIELD_WIDTH_METERS / 2) - DELIVERY_POSITION_Y_OFFSET_FROM_CENTER_METERS, true),
            LEFT_DELIVERY_POSITION = new FlippableTranslation2d(BLUE_RELATIVE_DELIVERY_POSITION_X, (FIELD_WIDTH_METERS / 2) + DELIVERY_POSITION_Y_OFFSET_FROM_CENTER_METERS, true);
    public static final double
            ALLIANCE_ZONE_LENGTH = 4,
            DELIVERY_ZONE_START_BLUE_X = ALLIANCE_ZONE_LENGTH + 1;

    private static AprilTagFieldLayout createAprilTagFieldLayout() {
        try {
            return SHOULD_USE_HOME_TAG_LAYOUT ?
                    new AprilTagFieldLayout(FilesHandler.DEPLOY_PATH + "field_calibration.json") :
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
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
}
