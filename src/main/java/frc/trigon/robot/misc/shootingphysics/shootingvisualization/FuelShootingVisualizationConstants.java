package frc.trigon.robot.misc.shootingphysics.shootingvisualization;

import edu.wpi.first.math.geometry.Translation3d;

public class FuelShootingVisualizationConstants {
    static final double G_FORCE = 9.81;
    static final double
            TOP_TRACTION_COEFFICIENT = 0.7,
            BOTTOM_TRACTION_COEFFICIENT = 1;
    static final double
            GAME_PIECE_MASS_KG = 0.2,
            GAME_PIECE_RADIUS_METERS = 0.075,
            GAME_PIECE_AREA = Math.PI * GAME_PIECE_RADIUS_METERS * GAME_PIECE_RADIUS_METERS,
            MOMENT_OF_INERTIA = 2.0 / 5.0 * GAME_PIECE_MASS_KG * GAME_PIECE_RADIUS_METERS * GAME_PIECE_RADIUS_METERS;
    static final double
            AIR_DENSITY = 1.205,
            DRAG_COEFFICIENT = 0.5,
            MAGNUS_LIFT_FACTOR = 0.6,
            SPIN_DECAY_COEFFICIENT = 0.01;
    static final Translation3d MAGNUS_SPIN_AXIS = new Translation3d(0, 1, 0);
    static final double TIME_STEP_SECONDS = 0.02;
}
