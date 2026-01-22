package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.ArrayList;
import java.util.List;

public class SimulatedGamePieceConstants {
    public static final double G_FORCE = 9.806;

    public static final double
            INTAKE_TOLERANCE_METERS = 0.3,
            LOADER_TOLERANCE_METERS = 0.1;

    static final int MAXIMUM_HELD_FUEL = 6;//TODO: Increase when capability for holding more fuel is implemented

    static final double FUEL_DIAMETER_METERS = 0.15;
    private static final int
            STARTING_FUEL_ROWS = 12,
            STARTING_FUEL_COLUMNS = 30;
    private static final double
            STARTING_FUEL_X_POSITION_METERS = 7.357364,
            STARTING_FUEL_Y_POSITION_METERS = 1.724406,
            STARTING_FUEL_SPACING_METERS = 0.16;
    static final Translation3d ROBOT_RELATIVE_HELD_FUEL_OFFSET_FROM_SPINDEXER_METERS = new Translation3d(
            0.165,
            0,
            0.2
    );

    static final Pose3d
            COLLECTION_CHECK_POSE = new Pose3d(0.7, 0, 0, new Rotation3d()),//TODO: Calibrate
            LOADER_CHECK_POSE = new Pose3d(0, 0.13455, 0.2, new Rotation3d());

    /**
     * Stores the game pieces that start on the field.
     * Replenishes as the game pieces are picked up.
     */
    public static final ArrayList<SimulatedGamePiece> STARTING_FUEL = getStartingFuel();

    private static ArrayList<SimulatedGamePiece> getStartingFuel() {
        final ArrayList<SimulatedGamePiece> startingFuel = new ArrayList<>(List.of());

        for (int i = 0; i < STARTING_FUEL_ROWS; i++) {
            for (int j = 0; j < STARTING_FUEL_COLUMNS; j++) {
                startingFuel.add(new SimulatedGamePiece(
                        STARTING_FUEL_X_POSITION_METERS + (i * STARTING_FUEL_SPACING_METERS),
                        STARTING_FUEL_Y_POSITION_METERS + (j * STARTING_FUEL_SPACING_METERS)
                ));
            }
        }

        return startingFuel;
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
