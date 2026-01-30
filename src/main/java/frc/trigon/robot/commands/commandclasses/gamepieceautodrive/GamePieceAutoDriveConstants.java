package frc.trigon.robot.commands.commandclasses.gamepieceautodrive;

import frc.trigon.robot.constants.FieldConstants;

public class GamePieceAutoDriveConstants {
    static final double CLUSTER_RADIUS_METERS = 1.0;
    static final double MAX_COLLECTION_X_METERS = FieldConstants.FIELD_LENGTH_METERS / 2.0;
    static final double SCORE_WEIGHT_COUNT = 2.5;
    static final double SCORE_PENALTY_DISTANCE = 1.0;
    static final double LINEARITY_THRESHOLD = 6.0;
    static final double BLEND_START_DISTANCE_METERS = 1.5;
    static final double BLEND_END_DISTANCE_METERS = 0.5;
}