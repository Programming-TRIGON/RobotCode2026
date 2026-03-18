package frc.trigon.robot.commands.commandclasses.gamepieceautodrive;

import frc.trigon.robot.constants.FieldConstants;

public class GamePieceAutoDriveConstants {
    static final double CLUSTER_RADIUS_METERS = 1.0;
    static final double MAX_COLLECTION_X_METERS = (FieldConstants.FIELD_LENGTH_METERS / 2.0) + 0.5; // 0.5m buffer beyond mid-field line
    static final double SCORE_WEIGHT_COUNT = 1;
    static final double SCORE_PENALTY_DISTANCE = 4;
    static final double LINEARITY_THRESHOLD = 6.0;
    static final double BLEND_START_DISTANCE_METERS = 1.5;
    static final double BLEND_END_DISTANCE_METERS = 0.5;

    // Intake slowdown
    public static final double INTAKE_SLOWDOWN_DISTANCE_METERS = 0.8;   // start ramping speed here
    public static final double INTAKE_DRIVE_SPEED_SCALE = 0.35;  // minimum speed multiplier at the piece

    // Rotation rate limit while intaking
    public static final double MAX_INTAKE_ROTATION_RATE_DEG_PER_SEC = 60.0;  // tune on-field

    public static final double ROBOT_HALF_WIDTH = 0.37;  // half-side of the robot square
    public static final double INTAKE_REACH = 0.4;  // how far the intake extends past the body
    public static final double ALLIANCE_WALL_X_METERS = 0.0;  // field X coordinate of the alliance wall TODO: THIS ASSUMES THE ALLIANCE IS BLUE?
    public static final double Y_WALL_PERPENDICULAR_TOLERANCE_METERS = 0.35;
}