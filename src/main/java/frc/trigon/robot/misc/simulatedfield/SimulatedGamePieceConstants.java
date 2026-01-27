package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.trigon.lib.utilities.flippable.FlippableTranslation3d;
import frc.trigon.robot.constants.FieldConstants;

public class SimulatedGamePieceConstants {
    public static final double SCORE_TOLERANCE_METERS = 0.3;
    static final double
            INTAKE_TOLERANCE_METERS = 0.3,
            LOADER_TOLERANCE_METERS = 0.05;
    public static final FlippableTranslation3d
            SCORE_CHECK_POSITION = new FlippableTranslation3d(new Translation3d(4.625594, FieldConstants.FIELD_WIDTH_METERS / 2, 1.4), true),
            EJECT_FUEL_FROM_HUB_POSITION = new FlippableTranslation3d(new Translation3d(5.189474, FieldConstants.FIELD_WIDTH_METERS / 2, 0.762), true);
    static final Translation3d
            COLLECTION_CHECK_POSITION = new Translation3d(0.4, 0, 0),
            LOADER_CHECK_POSITION = new Translation3d(0, 0.13455, 0.2);

    static final int MAXIMUM_HELD_FUEL = 40;
    static final Translation3d ROBOT_RELATIVE_HELD_FUEL_OFFSET_FROM_SPINDEXER_METERS = new Translation3d(
            0.165,
            0,
            0.2
    );

    static final Translation3d
            ROBOT_RELATIVE_HELD_UNINDEXED_FUEL_BOUNDING_BOX_START = new Translation3d(
            0.35,
            0.35,
            0.13
    ),
            ROBOT_RELATIVE_HELD_UNINDEXED_FUEL_BOUNDING_BOX_END = new Translation3d(
                    -0.35,
                    -0.35,
                    0.4
            );

    private static final int
            STARTING_FUEL_ROWS = 12,
            STARTING_FUEL_COLUMNS = 30;
    private static final double
            FUEL_DIAMETER_METERS = 0.15,
            STARTING_FUEL_X_POSITION_METERS = 7.357364,
            STARTING_FUEL_Y_POSITION_METERS = 1.724406,
            STARTING_FUEL_SPACING_METERS = 0.16;

    public static final double
            EJECTION_FROM_HUB_MINIMUM_VELOCITY_METERS_PER_SECOND = 4,
            EJECTION_FROM_HUB_MAXIMUM_VELOCITY_METERS_PER_SECOND = 15;
    public static final Rotation2d EJECTION_FROM_HUB_MAXIMUM_ANGLE = Rotation2d.fromDegrees(35);

    static {
        initializeFuel();
    }

    private static void initializeFuel() {
        for (int i = 0; i < STARTING_FUEL_ROWS; i++) {
            for (int j = 0; j < STARTING_FUEL_COLUMNS; j++) {
                new SimulatedGamePiece(
                        STARTING_FUEL_X_POSITION_METERS + (i * STARTING_FUEL_SPACING_METERS),
                        STARTING_FUEL_Y_POSITION_METERS + (j * STARTING_FUEL_SPACING_METERS)
                );
            }
        }
    }

    public enum GamePieceType {
        FUEL(FUEL_DIAMETER_METERS / 2.0, 0);

        public final double originPointHeightOffGroundMeters;
        public final int id;

        GamePieceType(double originPointHeightOffGroundMeters, int id) {
            this.originPointHeightOffGroundMeters = originPointHeightOffGroundMeters;
            this.id = id;
        }

        public static String getNameFromID(int id) {
            for (int i = 0; i < values().length; i++)
                if (values()[i].id == id)
                    return values()[i].toString();
            return "";
        }
    }
}
