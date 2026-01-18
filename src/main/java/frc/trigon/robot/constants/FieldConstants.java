package frc.trigon.robot.constants;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.lib.utilities.FilesHandler;
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

    public static final double
            HUB_X = 4.03,
            HUB_Y = FieldConstants.FIELD_WIDTH_METERS / 2,
            HUB_RADIUS_METERS = 1.19 / 2,
            HUB_LEFTMOST_Y = HUB_Y - HUB_RADIUS_METERS,
            HUB_RIGHTMOST_Y = HUB_Y + HUB_RADIUS_METERS;
    public static final double ALLIANCE_ZONE_LINE_X = HUB_X - HUB_RADIUS_METERS;
    public static final FlippableTranslation2d
            LEFT_DELIVERY_POSITION = new FlippableTranslation2d(new Translation2d(FieldConstants.ALLIANCE_ZONE_LINE_X / 2, FieldConstants.HUB_LEFTMOST_Y / 2), true),
            RIGHT_DELIVERY_POSITION = new FlippableTranslation2d(new Translation2d(FieldConstants.ALLIANCE_ZONE_LINE_X / 2, FieldConstants.HUB_LEFTMOST_Y * 1.5), true);

    private static final boolean SHOULD_USE_HOME_TAG_LAYOUT = false;
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = createAprilTagFieldLayout();
    private static final Transform3d TAG_OFFSET = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    public static final HashMap<Integer, Pose3d> TAG_ID_TO_POSE = fieldLayoutToTagIDToPoseMap();

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
